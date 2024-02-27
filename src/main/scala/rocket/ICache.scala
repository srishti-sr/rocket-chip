// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.
package freechips.rocketchip.rocket
import chisel3._
import chisel3.util.{Cat, Decoupled, Mux1H, OHToUInt, RegEnable, Valid, isPow2, log2Ceil, log2Up, PopCount}
import freechips.rocketchip.amba._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{DescribedSRAM, _}
import freechips.rocketchip.util.property
import chisel3.experimental.SourceInfo
import chisel3.dontTouch
import chisel3.util.random.LFSR
/* PipeLine:
  *   Stage 0 : access data and tag SRAM in parallel
  *   Stage 1 : receive paddr from CPU
  *             compare tag and paddr when the entry is valid
  *             if hit : pick up the target instruction
  *             if miss : start refilling in stage 2
  *   Stage 2 : respond to CPU or start a refill}}}*/
case class ICacheParams(
    nSets: Int = 128,
    nWays: Int = 2,
    rowBits: Int = 128,
    nTLBSets: Int = 1,
    nTLBWays: Int = 32,
    nTLBBasePageSectors: Int = 4,
    nTLBSuperpages: Int = 4,
    cacheIdBits: Int = 0,
    tagECC: Option[String] = None,
    dataECC: Option[String] = None,
    itimAddr: Option[BigInt] = None,
    prefetch: Boolean = false,
    blockBytes: Int = 64,
    latency: Int = 1,
    fetchBytes: Int = 4) extends L1CacheParams {
  def replacement = new RandomReplacement(nWays)
}
trait HasL1ICacheParameters extends HasL1CacheParameters with HasCoreParameters {
  val cacheParams = tileParams.icache.get   //L1 cache,base tile,core parameters
}
class ICacheReq(implicit p: Parameters) extends CoreBundle()(p) with HasL1ICacheParameters {
  val addr = UInt(vaddrBits.W)             //addressbits
}
class ICacheErrors(implicit p: Parameters) extends CoreBundle()(p)
    with HasL1ICacheParameters
    with CanHaveErrors {
  val bus = Valid(UInt(paddrBits.W))
}

class ICache(val icacheParams: ICacheParams, val staticIdForMetadataUseOnly: Int)(implicit p: Parameters) extends LazyModule {
  lazy val module = new ICacheModule(this)//use later
    /** Diplomatic hartid bundle used for ITIM.  */
  val hartIdSinkNodeOpt = icacheParams.itimAddr.map(_ => BundleBridgeSink[UInt]()) //why??
  /** @todo base address offset for ITIM? */
  val mmioAddressPrefixSinkNodeOpt = icacheParams.itimAddr.map(_ => BundleBridgeSink[UInt]()) //why??

  val useVM = p(TileKey).core.useVM   //use of virtual memory
    
  val masterNode = TLClientNode(Seq(TLMasterPortParameters.v1(      //tile link
    clients = Seq(TLMasterParameters.v1(
      sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint
      name = s"Core ${staticIdForMetadataUseOnly} ICache")), //what will come here??
    requestFields = useVM.option(Seq()).getOrElse(Seq(AMBAProtField())))))   //based on use of virtual memory
    
  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes  //size of cache
    
  //val itim_control_offset = size - icacheParams.nSets * icacheParams.blockBytes

  val device = new SimpleDevice("itim", Seq("sifive,itim0")) { 
    override def describe(resources: ResourceBindings): Description = {
     val Description(name, mapping) = super.describe(resources)
     val Seq(Binding(_, ResourceAddress(address, perms))) = resources("reg/mem")
     val base_address = address.head.base
     val mem_part = AddressSet.misaligned(base_address, itim_control_offset)
     val control_part = AddressSet.misaligned(base_address + itim_control_offset, size - itim_control_offset)
     val extra = Map(
       "reg-names" -> Seq(ResourceString("mem"), ResourceString("control")),
       "reg" -> Seq(ResourceAddress(mem_part, perms), ResourceAddress(control_part, perms)))
     Description(name, mapping ++ extra)
    }
  }
  /** @todo why [[wordBytes]] is defined by [[icacheParams.fetchBytes]], rather than 32 directly? */
  private val wordBytes = icacheParams.fetchBytes
  /** Instruction Tightly Integrated Memory node. */
  val slaveNode =
    TLManagerNode(icacheParams.itimAddr.toSeq.map { itimAddr => TLSlavePortParameters.v1( //tile link //how to do this without itim ??
      Seq(TLSlaveParameters.v1(
        address         = Seq(AddressSet(itimAddr, size-1)),
        resources       = device.reg("mem"),
        regionType      = RegionType.IDEMPOTENT,
        executable      = true,
        supportsPutFull = TransferSizes(1, wordBytes),
        supportsPutPartial = TransferSizes(1, wordBytes),
        supportsGet     = TransferSizes(1, wordBytes),
        fifoId          = Some(0))), // requests handled in FIFO order
      beatBytes = wordBytes,
      minLatency = 1)})
}
class ICacheResp(outer: ICache) extends Bundle {
  val data = UInt((outer.icacheParams.fetchBytes*8).W) //data to CPU
  val replay = False //used in case of tag error original Bool()
  val ae = Bool() //use??
}
class ICachePerfEvents extends Bundle { //performance counting
  val acquire = Bool() //cache aquires a cache line
}
//IO from CPU To Icache
class ICacheBundle(val outer: ICache) extends CoreBundle()(outer.p) { //core bundle parameters are available
  /** first cycle requested from CPU. */
  val req = Flipped(Decoupled(new ICacheReq)) //flip the direction of request from cpu to icache,Input port for instruction cache requests from the CPU.
  val s1_paddr = Input(UInt(paddrBits.W)) // delayed one cycle w.r.t. req ,first stage of pipeline
  val s2_vaddr = Input(UInt(vaddrBits.W)) // delayed two cycles w.r.t. req, second stage of pipeline
  val s1_kill = Input(Bool()) // delayed one cycle w.r.t. req,killing the request
  val s2_kill = Input(Bool()) // delayed two cycles; prevents I$ miss emission
  val s2_cacheable = Input(Bool()) /** Boolean indicating if a miss should be cached by the L2 cache.s an input signal indicating whether the corresponding instruction is cacheable at the L2 cache level.
It influences the caching behavior.,Flag indicating if a miss should be cached in the L2 cache, determined after processing in the second stage.**/
  val s2_prefetch = Input(Bool()) // should I$ prefetch next line on a miss?
  val resp = Valid(new ICacheResp(outer))
  val invalidate = Input(Bool()) //flush l1 cache of cpu
  val errors = new ICacheErrors
  val perf = Output(new ICachePerfEvents())//performance counting??
  val clock_enabled = Input(Bool())  /** enable clock. */
  /** I$ miss or ITIM access will still enable clock even [[ICache]] is asked to be gated. */
  val keep_clock_enabled = Output(Bool())
}
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
    with HasL1ICacheParameters {
  override val cacheParams = outer.icacheParams //use the local parameters
  val io = IO(new ICacheBundle(outer)) //IO between core and cache
        
  val (tl_out, edge_out) = outer.masterNode.out(0) /*Tile link port to memory,first ouput of the master node*/
   /** TileLink port as ITIM memory.
    * if [[outer.slaveNode]] is not connected [[outer.slaveNode.in]] will be empty.
    *
    * wes: Option.unzip does not exist :-(
    */
  val (tl_in, edge_in) = outer.slaveNode.in.headOption.unzip /**val (tl_in, edge_in) = outer.slaveNode.in.headOption.map(_.unzip).getOrElse((None, None))**/
//how to deal with this,in absence of itim??
  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode
  require(isPow2(nSets) && isPow2(nWays))
  require(!usingVM || outer.icacheParams.itimAddr.isEmpty || pgIdxBits >= untagBits,
    s"When VM and ITIM are enabled, I$$ set size must not exceed ${1<<(pgIdxBits-10)} KiB; got ${(outer.size/nWays)>>10} KiB")
  /** if this ICache can be used as ITIM, which hart it belongs to. */
        
  val io_hartid = outer.hartIdSinkNodeOpt.map(_.bundle)
  /** @todo tile Memory mapping I/O base address? */
  val io_mmio_address_prefix = outer.mmioAddressPrefixSinkNodeOpt.map(_.bundle)
  val scratchpadOn = RegInit(false.B)//register indicates wheather ITIM is enabled.          //what to do with scartchpad?
  val scratchpadMax = tl_in.map(tl => Reg(UInt(log2Ceil(nSets * (nWays - 1)).W)))  //a cut point to SRAM, indicates which SRAM will be used as SRAM or Cache.??
  /*def lineInScratchpad(line: UInt) = scratchpadMax.map(scratchpadOn && line <= _).getOrElse(false.B)
  val scratchpadBase = outer.icacheParams.itimAddr.map { dummy =>
    p(LookupByHartId)(_.icache.flatMap(_.itimAddr.map(_.U)), io_hartid.get) | io_mmio_address_prefix.get
  }
  def addrMaybeInScratchpad(addr: UInt) = scratchpadBase.map(base => addr >= base && addr < base + outer.size.U).getOrElse(false.B)
  def addrInScratchpad(addr: UInt) = addrMaybeInScratchpad(addr) && lineInScratchpad(addr(untagBits+log2Ceil(nWays)-1, blockOffBits))
  def scratchpadWay(addr: UInt) = addr.extract(untagBits+log2Ceil(nWays)-1, untagBits)
  def scratchpadWayValid(way: UInt) = way < (nWays - 1).U
  def scratchpadLine(addr: UInt) = addr(untagBits+log2Ceil(nWays)-1, blockOffBits)
*/
 //scratchpad access valid in stage
  val s0_slaveValid = tl_in.map(_.a.fire).getOrElse(false.B)
  val s1_slaveValid = RegNext(s0_slaveValid, false.B)
  val s2_slaveValid = RegNext(s1_slaveValid, false.B)
  val s3_slaveValid = RegNext(false.B)
        
  val s0_valid = io.req.fire //valid signal for CPU accessing cache in stage 0.
  val s0_vaddr = io.req.bits.addr //virtual address from CPU in stage 0.
  val s1_valid = RegInit(false.B)//valid signal for stage 1, drived by s0_valid.
  val s1_vaddr = RegEnable(s0_vaddr, s0_valid)//virtual address from CPU in stage 1 only when s0 is valid
  val s1_tag_hit = Wire(Vec(nWays, Bool()))/*tag hit vector to indicate hit which way, 
  Each element of the vector corresponds to a specific way in the cache, and the vector as a whole is intended to indicate which ways have a tag hit.*/ 
  val s1_hit = s1_tag_hit.reduce(_||_)
  dontTouch(s1_hit)
 // effectively performs a logical OR operation on all the elements of the s1_tag_hit vector, resulting in a single Boolean value
        
 /*RegNext is a built-in function used to create a register that holds the value of a signal at the next clock cycle*/
  val s2_valid = RegNext(s1_valid && !io.s1_kill, false.B) //s2_valid becomes true only if the data was valid in stage 1 and not killed by an external signal.
  val s2_hit = RegNext(s1_hit)//hold same value from stage 1
        
  val invalidated = Reg(Bool())//flush the cache status register
 /*RegInit is a function used to create a register with a specific initial value*/
  val refill_valid = RegInit(false.B)//Indicates if a refill request is currently active
  val send_hint = RegInit(false.B)// Indicates if a prefetch hint is being sent
  val refill_fire = tl_out.a.fire && !send_hint //indicate [[tl_out]] is performing a refill. 
  val hint_outstanding = RegInit(false.B)//Indicates if there's an outstanding prefetch hint
  val s2_miss = s2_valid && !s2_hit && !io.s2_kill//[[io]] access L1 I$ miss. 
  val s1_can_request_refill = !(s2_miss || refill_valid)//forward signal to stage 1, permit stage 1 refill
  /** real refill signal, stage 2 miss, and was permit to refill in stage 1.
    * Since a miss will trigger burst.
    * miss under miss won't trigger another burst.
    */
  val s2_request_refill = s2_miss && RegNext(s1_can_request_refill)
  val refill_paddr = RegEnable(io.s1_paddr, s1_valid && s1_can_request_refill)
  val refill_vaddr = RegEnable(s1_vaddr, s1_valid && s1_can_request_refill)
  val refill_tag = refill_paddr >> pgUntagBits
  val refill_idx = index(refill_vaddr, refill_paddr)
 /** AccessAckData, is refilling I$, it will block request from CPU. */
  val refill_one_beat = tl_out.d.fire && edge_out.hasData(tl_out.d.bits)

  io.req.ready := !(refill_one_beat)//block request from CPU when refill 
  s1_valid := s0_valid

  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
 /**The function returns a tuple of four values, out of which only two are captured:
d_done: A boolean signal indicating whether the last data beat has been reached.
refill_cnt: An integer representing the current refill count.**/
  val refill_done = refill_one_beat && d_done //at last beat of `tl_out.d.fire`, finish refill.
  require (edge_out.manager.minLatency > 0)

 /** way to be replaced, implemented with a hardcoded random replacement algorithm */
 val repl_way = if (isDM) 0.U else {
    // pick a way that is not used by the scratchpad
    val v= LFSR(16, refill_fire)(log2Up(nWays)-1,0)
    v
  }
/**  Tag SRAM, indexed with virtual memory,
 *   content with `refillError ## tag[19:0]` after ECC
 * */
  val tag_array  = DescribedSRAM(
    name = "tag_array",
    desc = "ICache Tag Array",
    size = nSets,
    data = Vec(nWays, UInt((tagBits).W))//changed
  )
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid)
     /**Read the tag array for the virtual index bits when no refill is being performed and s0 is valis */
  val accruedRefillError = Reg(Bool())
  val refillError = tl_out.d.bits.corrupt || (refill_cnt > 0.U && accruedRefillError)
  when (refill_done) {
    tag_array.write(refill_idx, VecInit(Seq.fill(nWays){refill_tag}), Seq.tabulate(nWays)(repl_way === _.U))
   /**Writing into the tag array the accessed data*/
    ccover(refillError, "D_CORRUPT", "I$ D-channel corrupt")
   /**Displaying the data has been corrupted*/
  }
  /**notify CPU, I$ has corrupt.*/
  io.errors.bus.valid := tl_out.d.fire && (tl_out.d.bits.denied || tl_out.d.bits.corrupt)
  io.errors.bus.bits  := (refill_paddr >> blockOffBits) << blockOffBits
  /**This operation likely provides the memory block number (excluding the offset)
     to the CPU as part of the error information. This helps the CPU identify the specific memory block associated with the potential ICache corruption.*/
  
  val vb_array = RegInit(0.U((nSets*nWays).W))
  /** true indicate this cacheline is valid,
    * indexed by (wayIndex ## setIndex)
    * after refill_done and not FENCE.I, (repl_way ## refill_idx) set to true.
    */
  when (refill_one_beat) {     /**when AccessAckData, is refilling I$*/
    accruedRefillError := refillError
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }
   /** flush cache when invalidate is true. */
  val invalidate = WireDefault(io.invalidate)
  when (invalidate) {
    vb_array := 0.U
    invalidated := true.B
  }
  val s1_tag_disparity = Wire(Vec(nWays, Bool()))/**True: Potentially correctable error (e.g., single-bit error).
False: Uncorrectable error (e.g., multiple-bit error).
Triggers actions for correctable errors:
CPU replay of instruction fetch.
ICache line invalidation.*/
  val s1_tl_error = Wire(Vec(nWays, Bool()))/**Signals uncorrectable errors on the ICache transaction bus:
True: Uncorrectable error detected.
False: No error detected.
Triggers error signaling to CPU and potentially other components:
Sets error bit (io.resp.bits.ae) in ICache response.
Signals error cause (Causes.fetch_access).*/
     
  val wordBits = outer.icacheParams.fetchBytes*8/**how many bits will be fetched by CPU for each fetch.*/
  val s1_dout = Wire(Vec(nWays, UInt(wordBits).W))) /** a set of raw data read from [[data_arrays]]. */
  s1_dout := DontCare
     
  val s0_slaveAddr = tl_in.map(_.a.bits.address).getOrElse(0.U)
  val s1s3_slaveAddr = Reg(UInt(log2Ceil(outer.size).W))
  val s1s3_slaveData = Reg(UInt(wordBits.W))
  for (i <- 0 until nWays) {
    val s1_idx = index(s1_vaddr, io.s1_paddr)
    val s1_tag = io.s1_paddr >> pgUntagBits
    val scratchpadHit = scratchpadWayValid(i.U) &&
      Mux(s1_slaveValid,
        lineInScratchpad(scratchpadLine(s1s3_slaveAddr)) && scratchpadWay(s1s3_slaveAddr) === i.U,
        addrInScratchpad(io.s1_paddr) && scratchpadWay(io.s1_paddr) === i.U)
    val s1_vb = vb_array(Cat(i.U, s1_idx))
    val enc_tag = tECC.decode(tag_rdata(i))
    val (tl_error, tag) = Split(enc_tag.uncorrected, tagBits)
    val tagMatch = s1_vb && tag === s1_tag
    1_tag_disparity(i) := s1_vb && enc_tag.error
    s1_tl_error(i) := tagMatch && tl_error.asBool?
    s1_tag_hit(i) := tagMatch
  }
  assert(!(s1_valid || s1_slaveValid) || PopCount(s1_tag_hit zip s1_tag_disparity map { case (h, d) => h && !d }) <= 1.U)//work on popcount
  require(tl_out.d.bits.data.getWidth % wordBits == 0)

    /**Data SRAM**/
  val data_arrays = Seq.tabulate(tl_out.d.bits.data.getWidth / wordBits) {
    i =>
      DescribedSRAM(
        name = s"data_arrays_${i}",
        desc = "ICache Data Array",
        size = nSets * refillCycles,
        data = Vec(nWays, UInt(dECC.width(wordBits).W))
      )
  }
  for ((data_array , i) <- data_arrays.zipWithIndex) {
    def wordMatch(addr: UInt) = addr.extract(log2Ceil(tl_out.d.bits.data.getWidth/8)-1, log2Ceil(wordBits/8)) === i.U
    def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
    val s0_ren = (s0_valid && wordMatch(s0_vaddr)) || (s0_slaveValid && wordMatch(s0_slaveAddr))
    val wen = (refill_one_beat && !invalidated) || (s3_slaveValid && wordMatch(s1s3_slaveAddr))
    val mem_idx =
      Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                  Mux(s3_slaveValid, row(s1s3_slaveAddr),
                  Mux(s0_slaveValid, row(s0_slaveAddr),
                  row(s0_vaddr))))
    when (wen) {
      val data = Mux(s3_slaveValid, s1s3_slaveData, tl_out.d.bits.data(wordBits*(i+1)-1, wordBits*i))
      val way = Mux(s3_slaveValid, scratchpadWay(s1s3_slaveAddr), repl_way)
      data_array.write(mem_idx, VecInit(Seq.fill(nWays){dECC.encode(data)}), (0 until nWays).map(way === _.U))
    }
    val dout = data_array.read(mem_idx, !wen && s0_ren)
    when (wordMatch(Mux(s1_slaveValid, s1s3_slaveAddr, io.s1_paddr))) {
      s1_dout := dout
    }
  }
      require(dECC.isInstanceOf[IdentityCode])
      require(outer.icacheParams.itimAddr.isEmpty)
      io.resp.bits.data := Mux1H(s1_tag_hit, s1_dout)
      //io.resp.bits.ae := s1_tl_error.asUInt.orR
      io.resp.valid := s1_valid && s1_hit
      io.resp.bits.replay := false.B
  tl_out.a.valid := s2_request_refill
  tl_out.a.bits := edge_out.Get(
                    fromSource = 0.U,
                    toAddress = (refill_paddr >> blockOffBits) << blockOffBits,
                    lgSize = lgCacheBlockBytes.U)._2
  
  tl_out.a.bits.user.lift(AMBAProt).foreach { x =>
    x.fetch       := true.B
    x.secure      := true.B
    x.privileged  := true.B
    x.bufferable  := true.B
    x.modifiable  := true.B
    x.readalloc   := io.s2_cacheable
    x.writealloc  := io.s2_cacheable
  }
  tl_out.b.ready := true.B
  tl_out.c.valid := false.B
  tl_out.e.valid := false.B
  assert(!(tl_out.a.valid))
  when (!refill_valid) { invalidated := false.B }
  when (refill_fire) { refill_valid := true.B }
  when (refill_done) { refill_valid := false.B}
  io.perf.acquire := refill_fire
  io.keep_clock_enabled :=
    s1_valid || s2_valid || refill_valid || send_hint || hint_outstanding // I$
    
  def index(vaddr: UInt, paddr: UInt) = { //VIPT
    val lsbs = paddr(pgUntagBits-1, blockOffBits)//LSB using physical address
    val msbs = (idxBits+blockOffBits > pgUntagBits).option(vaddr(idxBits+blockOffBits-1, pgUntagBits))//if not used returns none 
    msbs ## lsbs //concatination to form the indexing bits
  }
  ccover(!send_hint && (tl_out.a.valid && !tl_out.a.ready), "MISS_A_STALL", "I$ miss blocked by A-channel")
  ccover(invalidate && refill_valid, "FLUSH_DURING_MISS", "I$ flushed during miss")
  def ccover(cond: Bool, label: String, desc: String)(implicit sourceInfo: SourceInfo) =
    property.cover(cond, s"ICACHE_$label", "MemorySystem;;" + desc)
  val mem_active_valid = Seq(property.CoverBoolean(s2_valid, Seq("mem_active")))
 val data_error = Seq(
    property.CoverBoolean(!s2_data_decoded.correctable && !s2_data_decoded.uncorrectable, Seq("no_data_error")),
    property.CoverBoolean(s2_data_decoded.correctable, Seq("data_correctable_error")),
    property.CoverBoolean(s2_data_decoded.uncorrectable, Seq("data_uncorrectable_error")))
  val request_source = Seq(
    property.CoverBoolean(!s2_slaveValid, Seq("from_CPU")),
    property.CoverBoolean(s2_slaveValid, Seq("from_TL"))
  )
  val tag_error = Seq(
    property.CoverBoolean(!s2_tag_disparity, Seq("no_tag_error")),
    property.CoverBoolean(s2_tag_disparity, Seq("tag_error"))
  )
  val mem_mode = Seq(
    property.CoverBoolean(s2_scratchpad_hit, Seq("ITIM_mode")),
    property.CoverBoolean(!s2_scratchpad_hit, Seq("cache_mode"))
  )

  val error_cross_covers = new property.CrossProperty(
    Seq(mem_active_valid, data_error, tag_error, request_source, mem_mode),
    Seq(
      // tag error cannot occur in ITIM mode
      Seq("tag_error", "ITIM_mode"),
      // Can only respond to TL in ITIM mode
      Seq("from_TL", "cache_mode")
    ),
    "MemorySystem;;Memory Bit Flip Cross Covers")

  property.cover(error_cross_covers)
}
