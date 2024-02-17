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
case class ICacheParams(
    nSets: Int = 64,
    nWays: Int = 4,
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
    latency: Int = 2,
    fetchBytes: Int = 4) extends L1CacheParams {
  def replacement = new RandomReplacement(nWays)
}
trait HasL1ICacheParameters extends HasL1CacheParameters with HasCoreParameters {
  val cacheParams = tileParams.icache.get   //L1 cache,base tile,core parameters
}
class ICacheReq(implicit p: Parameters) extends CoreBundle()(p) with HasL1ICacheParameters {
  val addr = UInt(vaddrBits.W)             //addressbits
}
class ICache(val icacheParams: ICacheParams, val staticIdForMetadataUseOnly: Int)(implicit p: Parameters) extends LazyModule {
  lazy val module = new ICacheModule(this)
  val useVM = p(TileKey).core.useVM
  val masterNode = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
      sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint
      name = s"Core ${staticIdForMetadataUseOnly} ICache")),
    requestFields = useVM.option(Seq()).getOrElse(Seq(AMBAProtField())))))
  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  val itim_control_offset = size - icacheParams.nSets * icacheParams.blockBytes

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
    TLManagerNode(icacheParams.itimAddr.toSeq.map { itimAddr => TLSlavePortParameters.v1(
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
  val data = UInt((outer.icacheParams.fetchBytes*8).W)
  val replay = Bool()
  val ae = Bool()
}
class ICachePerfEvents extends Bundle {
  val acquire = Bool()
}
class ICacheBundle(val outer: ICache) extends CoreBundle()(outer.p) {
  val req = Flipped(Decoupled(new ICacheReq))
  val s1_paddr = Input(UInt(paddrBits.W)) 
  val s2_vaddr = Input(UInt(vaddrBits.W)) 
  val s1_kill = Input(Bool()) 
  val s2_kill = Input(Bool()) // delayed two cycles; prevents I$ miss emission
  val s2_cacheable = Input(Bool()) // should L2 cache line on a miss?
  val s2_prefetch = Input(Bool()) // should I$ prefetch next line on a miss?
  val resp = Valid(new ICacheResp(outer))
  val invalidate = Input(Bool())
  val errors = new ICacheErrors
  val perf = Output(new ICachePerfEvents())
  val clock_enabled = Input(Bool())
  val keep_clock_enabled = Output(Bool())
}
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
    with HasL1ICacheParameters {
  override val cacheParams = outer.icacheParams 
  val io = IO(new ICacheBundle(outer))
  val (tl_out, edge_out) = outer.masterNode.out(0)
  val (tl_in, edge_in) = outer.slaveNode.in.headOption.unzip
  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode
  require(isPow2(nSets) && isPow2(nWays))
  require(!usingVM || outer.icacheParams.itimAddr.isEmpty || pgIdxBits >= untagBits,
    s"When VM and ITIM are enabled, I$$ set size must not exceed ${1<<(pgIdxBits-10)} KiB; got ${(outer.size/nWays)>>10} KiB")
  val io_hartid = outer.hartIdSinkNodeOpt.map(_.bundle)
  val io_mmio_address_prefix = outer.mmioAddressPrefixSinkNodeOpt.map(_.bundle)
  val scratchpadOn = RegInit(false.B)
  val scratchpadMax = tl_in.map(tl => Reg(UInt(log2Ceil(nSets * (nWays - 1)).W)))
  def lineInScratchpad(line: UInt) = scratchpadMax.map(scratchpadOn && line <= _).getOrElse(false.B)
  val scratchpadBase = outer.icacheParams.itimAddr.map { dummy =>
    p(LookupByHartId)(_.icache.flatMap(_.itimAddr.map(_.U)), io_hartid.get) | io_mmio_address_prefix.get
  }
  def addrMaybeInScratchpad(addr: UInt) = scratchpadBase.map(base => addr >= base && addr < base + outer.size.U).getOrElse(false.B)
  def addrInScratchpad(addr: UInt) = addrMaybeInScratchpad(addr) && lineInScratchpad(addr(untagBits+log2Ceil(nWays)-1, blockOffBits))
  def scratchpadWay(addr: UInt) = addr.extract(untagBits+log2Ceil(nWays)-1, untagBits)
  def scratchpadWayValid(way: UInt) = way < (nWays - 1).U
  def scratchpadLine(addr: UInt) = addr(untagBits+log2Ceil(nWays)-1, blockOffBits)
  val s0_slaveValid = tl_in.map(_.a.fire).getOrElse(false.B)
  val s1_slaveValid = RegNext(s0_slaveValid, false.B)
  val s2_slaveValid = RegNext(s1_slaveValid, false.B)
  val s3_slaveValid = RegNext(false.B)
  val s0_valid = io.req.fire
  val s0_vaddr = io.req.bits.addr
  val s1_valid = RegInit(false.B)
  val s1_vaddr = RegEnable(s0_vaddr, s0_valid)
  val s1_tag_hit = Wire(Vec(nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_) || Mux(s1_slaveValid, true.B, addrMaybeInScratchpad(io.s1_paddr))
  dontTouch(s1_hit)
  val s2_valid = RegNext(s1_valid && !io.s1_kill, false.B)
  val s2_hit = RegNext(s1_hit)
  val invalidated = Reg(Bool())
  val refill_valid = RegInit(false.B)
  val send_hint = RegInit(false.B)
  val refill_fire = tl_out.a.fire && !send_hint
  val hint_outstanding = RegInit(false.B)
  val s2_miss = s2_valid && !s2_hit && !io.s2_kill
  val s1_can_request_refill = !(s2_miss || refill_valid)
  val s2_request_refill = s2_miss && RegNext(s1_can_request_refill)
  val refill_paddr = RegEnable(io.s1_paddr, s1_valid && s1_can_request_refill)
  val refill_vaddr = RegEnable(s1_vaddr, s1_valid && s1_can_request_refill)
  val refill_tag = refill_paddr >> pgUntagBits
  val refill_idx = index(refill_vaddr, refill_paddr)
  val refill_one_beat = tl_out.d.fire && edge_out.hasData(tl_out.d.bits)
  io.req.ready := !(refill_one_beat || s0_slaveValid || s3_slaveValid)
  s1_valid := s0_valid
  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
  val refill_done = refill_one_beat && d_done
  tl_out.d.ready := !s3_slaveValid
  require (edge_out.manager.minLatency > 0)
  val repl_way = if (isDM) 0.U else {
    val v0 = LFSR(16, refill_fire)(log2Up(nWays)-1,0)
    var v = v0
    for (i <- log2Ceil(nWays) - 1 to 0 by -1) {
      val mask = nWays - (BigInt(1) << (i + 1))
      v = v | (lineInScratchpad(Cat(v0 | mask.U, refill_idx)) << i)
    }
    assert(!lineInScratchpad(Cat(v, refill_idx)))
    v
  }
  val tag_array  = DescribedSRAM(
    name = "tag_array",
    desc = "ICache Tag Array",
    size = nSets,
    data = Vec(nWays, UInt(tECC.width(1 + tagBits).W))
  )
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1,blockOffBits), !refill_done && s0_valid)
  val accruedRefillError = Reg(Bool())
  val refillError = tl_out.d.bits.corrupt || (refill_cnt > 0.U && accruedRefillError)
  when (refill_done) {
    val enc_tag = tECC.encode(Cat(refillError, refill_tag))
    tag_array.write(refill_idx, VecInit(Seq.fill(nWays){enc_tag}), Seq.tabulate(nWays)(repl_way === _.U))

    ccover(refillError, "D_CORRUPT", "I$ D-channel corrupt")
  }
  io.errors.bus.valid := tl_out.d.fire && (tl_out.d.bits.denied || tl_out.d.bits.corrupt)
  io.errors.bus.bits  := (refill_paddr >> blockOffBits) << blockOffBits
  val vb_array = RegInit(0.U((nSets*nWays).W))
  when (refill_one_beat) {
    accruedRefillError := refillError
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }
  val invalidate = WireDefault(io.invalidate)
  when (invalidate) {
    vb_array := 0.U
    invalidated := true.B
  }
  val s1_tag_disparity = Wire(Vec(nWays, Bool()))
  val s1_tl_error = Wire(Vec(nWays, Bool()))
  val wordBits = outer.icacheParams.fetchBytes*8
  val s1_dout = Wire(Vec(nWays, UInt(dECC.width(wordBits).W)))
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
    val s1_vb = vb_array(Cat(i.U, s1_idx)) && !s1_slaveValid
    val enc_tag = tECC.decode(tag_rdata(i))
    val (tl_error, tag) = Split(enc_tag.uncorrected, tagBits)
    val tagMatch = s1_vb && tag === s1_tag
    s1_tag_disparity(i) := s1_vb && enc_tag.error
    s1_tl_error(i) := tagMatch && tl_error.asBool
    s1_tag_hit(i) := tagMatch || scratchpadHit
  }
  assert(!(s1_valid || s1_slaveValid) || PopCount(s1_tag_hit zip s1_tag_disparity map { case (h, d) => h && !d }) <= 1.U)
  require(tl_out.d.bits.data.getWidth % wordBits == 0)
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
  val s1s2_full_word_write = WireDefault(false.B)
  val s1_dont_read = s1_slaveValid && s1s2_full_word_write
  val s1_clk_en = s1_valid || s1_slaveValid
  val s2_tag_hit = RegEnable(Mux(s1_dont_read, 0.U.asTypeOf(s1_tag_hit), s1_tag_hit), s1_clk_en)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s2_scratchpad_word_addr = Cat(s2_hit_way, Mux(s2_slaveValid, s1s3_slaveAddr, io.s2_vaddr)(untagBits-1, log2Ceil(wordBits/8)), 0.U(log2Ceil(wordBits/8).W))
  val s2_dout = RegEnable(s1_dout, s1_clk_en)
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)
  val s2_tag_disparity = RegEnable(s1_tag_disparity, s1_clk_en).asUInt.orR
  val s2_tl_error = RegEnable(s1_tl_error.asUInt.orR, s1_clk_en)
  val s2_data_decoded = dECC.decode(s2_way_mux)
  val s2_disparity = s2_tag_disparity || s2_data_decoded.error
  val s1_scratchpad_hit = Mux(s1_slaveValid, lineInScratchpad(scratchpadLine(s1s3_slaveAddr)), addrInScratchpad(io.s1_paddr))
  val s2_scratchpad_hit = RegEnable(s1_scratchpad_hit, s1_clk_en)
  val s2_report_uncorrectable_error = s2_scratchpad_hit && s2_data_decoded.uncorrectable && (s2_valid || (s2_slaveValid && !s1s2_full_word_write))
  val s2_error_addr = scratchpadBase.map(base => Mux(s2_scratchpad_hit, base + s2_scratchpad_word_addr, 0.U)).getOrElse(0.U)
  outer.icacheParams.latency match {
    case 1 =>
      require(tECC.isInstanceOf[IdentityCode])
      require(dECC.isInstanceOf[IdentityCode])
      require(outer.icacheParams.itimAddr.isEmpty)
      io.resp.bits.data := Mux1H(s1_tag_hit, s1_dout)
      io.resp.bits.ae := s1_tl_error.asUInt.orR
      io.resp.valid := s1_valid && s1_hit
      io.resp.bits.replay := false.B
      case 2 =>
     /* when (s2_valid && s2_disparity) { invalidate := true.B }
      io.resp.bits.data := s2_data_decoded.uncorrected
      io.resp.bits.ae := s2_tl_error
      io.resp.bits.replay := s2_disparity
      io.resp.valid := s2_valid && s2_hit
      io.errors.correctable.foreach { c =>
        c.valid := (s2_valid || s2_slaveValid) && s2_disparity && !s2_report_uncorrectable_error
        c.bits := s2_error_addr
      }
      io.errors.uncorrectable.foreach { u =>
        u.valid := s2_report_uncorrectable_error
        u.bits := s2_error_addr
      }
      tl_in.map { tl =>
        val respValid = RegInit(false.B)
        tl.a.ready := !(tl_out.d.valid || s1_slaveValid || s2_slaveValid || s3_slaveValid || respValid || !io.clock_enabled)
        val s1_a = RegEnable(tl.a.bits, s0_slaveValid)
        s1s2_full_word_write := edge_in.get.hasData(s1_a) && s1_a.mask.andR
        when (s0_slaveValid) {
          val a = tl.a.bits
          s1s3_slaveAddr := tl.a.bits.address
          s1s3_slaveData := tl.a.bits.data
          when (edge_in.get.hasData(a)) {
            val enable = scratchpadWayValid(scratchpadWay(a.address))
            when (!lineInScratchpad(scratchpadLine(a.address))) {
              scratchpadMax.get := scratchpadLine(a.address)
              invalidate := true.B
            }
            scratchpadOn := enable
            val itim_allocated = !scratchpadOn && enable
            val itim_deallocated = scratchpadOn && !enable
            val itim_increase = scratchpadOn && enable && scratchpadLine(a.address) > scratchpadMax.get
            val refilling = refill_valid && refill_cnt > 0.U
            ccover(itim_allocated, "ITIM_ALLOCATE", "ITIM allocated")
            ccover(itim_allocated && refilling, "ITIM_ALLOCATE_WHILE_REFILL", "ITIM allocated while I$ refill")
            ccover(itim_deallocated, "ITIM_DEALLOCATE", "ITIM deallocated")
            ccover(itim_deallocated && refilling, "ITIM_DEALLOCATE_WHILE_REFILL", "ITIM deallocated while I$ refill")
            ccover(itim_increase, "ITIM_SIZE_INCREASE", "ITIM size increased")
            ccover(itim_increase && refilling, "ITIM_SIZE_INCREASE_WHILE_REFILL", "ITIM size increased while I$ refill")
          }
        }
        assert(!s2_valid || RegNext(RegNext(s0_vaddr)) === io.s2_vaddr)
        when (!(tl.a.valid || s1_slaveValid || s2_slaveValid || respValid)
              && s2_valid && s2_data_decoded.error && !s2_tag_disparity) {
          s3_slaveValid := true.B
          s1s3_slaveData := s2_data_decoded.corrected
          s1s3_slaveAddr := s2_scratchpad_word_addr | s1s3_slaveAddr(log2Ceil(wordBits/8)-1, 0)
        }
        respValid := s2_slaveValid || (respValid && !tl.d.ready) 
        val respError = RegEnable(s2_scratchpad_hit && s2_data_decoded.uncorrectable && !s1s2_full_word_write, s2_slaveValid)
        when (s2_slaveValid) {
          when (edge_in.get.hasData(s1_a) || s2_data_decoded.error) { s3_slaveValid := true.B }
          def byteEn(i: Int) = !(edge_in.get.hasData(s1_a) && s1_a.mask(i))
          s1s3_slaveData := (0 until wordBits/8).map(i => Mux(byteEn(i), s2_data_decoded.corrected, s1s3_slaveData)(8*(i+1)-1, 8*i)).asUInt
        }
        tl.d.valid := respValid
        tl.d.bits := Mux(edge_in.get.hasData(s1_a),
          edge_in.get.AccessAck(s1_a),
          edge_in.get.AccessAck(s1_a, 0.U, denied = false.B, corrupt = respError))
        tl.d.bits.data := s1s3_slaveData
        tl.b.valid := false.B
        tl.c.ready := true.B
        tl.e.ready := true.B
        ccover(s0_valid && s1_slaveValid, "CONCURRENT_ITIM_ACCESS_1", "ITIM accessed, then I$ accessed next cycle")
        ccover(s0_valid && s2_slaveValid, "CONCURRENT_ITIM_ACCESS_2", "ITIM accessed, then I$ accessed two cycles later")
        ccover(tl.d.valid && !tl.d.ready, "ITIM_D_STALL", "ITIM response blocked by D-channel")
        ccover(tl_out.d.valid && !tl_out.d.ready, "ITIM_BLOCK_D", "D-channel blocked by ITIM access")
      }*/
  }
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
  assert(!(tl_out.a.valid && addrMaybeInScratchpad(tl_out.a.bits.address)))
  when (!refill_valid) { invalidated := false.B }
  when (refill_fire) { refill_valid := true.B }
  when (refill_done) { refill_valid := false.B}
  io.perf.acquire := refill_fire
  io.keep_clock_enabled :=
    tl_in.map(tl => tl.a.valid || tl.d.valid || s1_slaveValid || s2_slaveValid || s3_slaveValid).getOrElse(false.B) || // ITIM
    s1_valid || s2_valid || refill_valid || send_hint || hint_outstanding // I$
  def index(vaddr: UInt, paddr: UInt) = {
    val lsbs = paddr(pgUntagBits-1, blockOffBits)
    val msbs = (idxBits+blockOffBits > pgUntagBits).option(vaddr(idxBits+blockOffBits-1, pgUntagBits))
    msbs ## lsbs
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
      Seq("tag_error", "ITIM_mode"),
      Seq("from_TL", "cache_mode")
    ),
    "MemorySystem;;Memory Bit Flip Cross Covers")
  property.cover(error_cross_covers)
}
