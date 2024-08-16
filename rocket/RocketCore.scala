// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import chisel3.withClock
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property
import freechips.rocketchip.scie._
import scala.collection.mutable.ArrayBuffer

case class RocketCoreParams(
  bootFreqHz: BigInt = 0,
  useVM: Boolean = true,
  useUser: Boolean = false,
  useSupervisor: Boolean = false,
  useHypervisor: Boolean = false,
  useDebug: Boolean = true,
  useAtomics: Boolean = true,
  useAtomicsOnlyForIO: Boolean = false,
  useCompressed: Boolean = true,
  useRVE: Boolean = false,
  useSCIE: Boolean = false,
  useBitManip: Boolean = false,
  useBitManipCrypto: Boolean = false,
  useCryptoNIST: Boolean = false,
  useCryptoSM: Boolean = false,
  useConditionalZero: Boolean = false,
  nLocalInterrupts: Int = 0,
  useNMI: Boolean = false,
  nBreakpoints: Int = 1,
  useBPWatch: Boolean = false,
  mcontextWidth: Int = 0,
  scontextWidth: Int = 0,
  nPMPs: Int = 8,
  nPerfCounters: Int = 0,
  haveBasicCounters: Boolean = true,
  haveCFlush: Boolean = false,
  misaWritable: Boolean = true,
  nL2TLBEntries: Int = 0,
  nL2TLBWays: Int = 1,
  nPTECacheEntries: Int = 8,
  mtvecInit: Option[BigInt] = Some(BigInt(0)),
  mtvecWritable: Boolean = true,
  fastLoadWord: Boolean = true,
  fastLoadByte: Boolean = false,
  branchPredictionModeCSR: Boolean = false,
  clockGate: Boolean = false,
  mvendorid: Int = 0, // 0 means non-commercial implementation
  mimpid: Int = 0x20181004, // release date in BCD
  mulDiv: Option[MulDivParams] = Some(MulDivParams()),
  fpu: Option[FPUParams] = Some(FPUParams()),
  haveCease: Boolean = true, // non-standard CEASE instruction
  debugROB: Boolean = false // if enabled, uses a C++ debug ROB to generate trace-with-wdata
) extends CoreParams {
  val lgPauseCycles = 5
  val haveFSDirty = false
  val pmpGranularity: Int = if (useHypervisor) 4096 else 4
  val fetchWidth: Int = if (useCompressed) 2 else 1
  //  fetchWidth doubled, but coreInstBytes halved, for RVC:
  val decodeWidth: Int = fetchWidth / (if (useCompressed) 2 else 1)
  val retireWidth: Int = 1
  val instBits: Int = if (useCompressed) 16 else 32
  val lrscCycles: Int = 80 // worst case is 14 mispredicted branches + slop
  val traceHasWdata: Boolean = false // ooo wb, so no wdata in trace
  override val customIsaExt = Option.when(haveCease)("xrocket") // CEASE instruction
  override def minFLen: Int = fpu.map(_.minFLen).getOrElse(32)
  override def customCSRs(implicit p: Parameters) = new RocketCustomCSRs
}

trait HasRocketCoreParameters extends HasCoreParameters {
  lazy val rocketParams: RocketCoreParams = tileParams.core.asInstanceOf[RocketCoreParams]

  val fastLoadWord = rocketParams.fastLoadWord
  val fastLoadByte = rocketParams.fastLoadByte

  val mulDivParams = rocketParams.mulDiv.getOrElse(MulDivParams()) // TODO ask andrew about this

  val usingABLU = usingBitManip || usingBitManipCrypto
  val aluFn = if (usingABLU) new ABLUFN else new ALUFN

  require(!fastLoadByte || fastLoadWord)
  require(!rocketParams.haveFSDirty, "rocket doesn't support setting fs dirty from outside, please disable haveFSDirty")
  require(!(usingABLU && usingConditionalZero), "Zicond is not yet implemented in ABLU")
}

class RocketCustomCSRs(implicit p: Parameters) extends CustomCSRs with HasRocketCoreParameters {
  override def bpmCSR = {
    rocketParams.branchPredictionModeCSR.option(CustomCSR(bpmCSRId, BigInt(1), Some(BigInt(0))))
  }

  private def haveDCache = tileParams.dcache.get.scratch.isEmpty

  override def chickenCSR = {
    val mask = BigInt(
      tileParams.dcache.get.clockGate.toInt << 0 |
      rocketParams.clockGate.toInt << 1 |
      rocketParams.clockGate.toInt << 2 |
      1 << 3 | // disableSpeculativeICacheRefill
      haveDCache.toInt << 9 | // suppressCorruptOnGrantData
      tileParams.icache.get.prefetch.toInt << 17
    )
    Some(CustomCSR(chickenCSRId, mask, Some(mask)))
  }

  def disableICachePrefetch = getOrElse(chickenCSR, _.value(17), true.B)

  def marchid = CustomCSR.constant(CSRs.marchid, BigInt(1))

  def mvendorid = CustomCSR.constant(CSRs.mvendorid, BigInt(rocketParams.mvendorid))

  // mimpid encodes a release version in the form of a BCD-encoded datestamp.
  def mimpid = CustomCSR.constant(CSRs.mimpid, BigInt(rocketParams.mimpid))

  override def decls = super.decls :+ marchid :+ mvendorid :+ mimpid
}

class Rocket(tile: RocketTile)(implicit p: Parameters) extends CoreModule()(p)
    with HasRocketCoreParameters
    with HasCoreIO {
  def nTotalRoCCCSRs = tile.roccCSRs.flatten.size

  val clock_en_reg = RegInit(true.B)
  val long_latency_stall = Reg(Bool())
  val id_reg_pause = Reg(Bool())
  val imem_might_request_reg = Reg(Bool())
  val clock_en = WireDefault(true.B)
  val gated_clock =
    if (!rocketParams.clockGate) clock
    else ClockGate(clock, clock_en, "rocket_clock_gate")

  class RocketImpl { // entering gated-clock domain

  // performance counters
  def pipelineIDToWB[T <: Data](x: T): T =
    RegEnable(RegEnable(RegEnable(x, !ctrl_killd), ex_pc_valid), mem_pc_valid)
  val perfEvents = new EventSets(Seq(
    new EventSet((mask, hits) => Mux(wb_xcpt, mask(0), wb_valid && pipelineIDToWB((mask & hits).orR)), Seq(
      ("exception", () => false.B),
      ("load", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp),
      ("store", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp),
      ("amo", () => usingAtomics.B && id_ctrl.mem && (isAMO(id_ctrl.mem_cmd) || id_ctrl.mem_cmd.isOneOf(M_XLR, M_XSC))),
      ("system", () => id_ctrl.csr =/= CSR.N),
      ("arith", () => id_ctrl.wxd && !(id_ctrl.jal || id_ctrl.jalr || id_ctrl.mem || id_ctrl.fp || id_ctrl.mul || id_ctrl.div || id_ctrl.csr =/= CSR.N)),
      ("branch", () => id_ctrl.branch),
      ("jal", () => id_ctrl.jal),
      ("jalr", () => id_ctrl.jalr))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul", () => if (pipelinedMul) id_ctrl.mul else id_ctrl.div && (id_ctrl.alu_fn & aluFn.FN_DIV) =/= aluFn.FN_DIV),
        ("div", () => if (pipelinedMul) id_ctrl.div else id_ctrl.div && (id_ctrl.alu_fn & aluFn.FN_DIV) === aluFn.FN_DIV)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp load", () => id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen),
        ("fp store", () => id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen),
        ("fp add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.swap23),
        ("fp mul", () => id_ctrl.fp && io.fpu.dec.fma && !io.fpu.dec.swap23 && !io.fpu.dec.ren3),
        ("fp mul-add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.ren3),
        ("fp div/sqrt", () => id_ctrl.fp && (io.fpu.dec.div || io.fpu.dec.sqrt)),
        ("fp other", () => id_ctrl.fp && !(io.fpu.dec.ldst || io.fpu.dec.fma || io.fpu.dec.div || io.fpu.dec.sqrt))))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("load-use interlock", () => id_ex_hazard && ex_ctrl.mem || id_mem_hazard && mem_ctrl.mem || id_wb_hazard && wb_ctrl.mem),
      ("long-latency interlock", () => id_sboard_hazard),
      ("csr interlock", () => id_ex_hazard && ex_ctrl.csr =/= CSR.N || id_mem_hazard && mem_ctrl.csr =/= CSR.N || id_wb_hazard && wb_ctrl.csr =/= CSR.N),
      ("I$ blocked", () => icache_blocked),
      ("D$ blocked", () => id_ctrl.mem && dcache_blocked),
      ("branch misprediction", () => take_pc_mem && mem_direction_misprediction),
      ("control-flow target misprediction", () => take_pc_mem && mem_misprediction && mem_cfi && !mem_direction_misprediction && !icache_blocked),
      ("flush", () => wb_reg_flush_pipe),
      ("replay", () => replay_wb))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul/div interlock", () => id_ex_hazard && (ex_ctrl.mul || ex_ctrl.div) || id_mem_hazard && (mem_ctrl.mul || mem_ctrl.div) || id_wb_hazard && wb_ctrl.div)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp interlock", () => id_ex_hazard && ex_ctrl.fp || id_mem_hazard && mem_ctrl.fp || id_wb_hazard && wb_ctrl.fp || id_ctrl.fp && id_stall_fpu)))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss", () => io.imem.perf.acquire),
      ("D$ miss", () => io.dmem.perf.acquire),
      ("D$ release", () => io.dmem.perf.release),
      ("ITLB miss", () => io.imem.perf.tlbMiss),
      ("DTLB miss", () => io.dmem.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))

/*runahead code begin*/
      dontTouch(io.dmem.perf)
      dontTouch(io.ptw.perf)
/*runahead code end*/

  val pipelinedMul = usingMulDiv && mulDivParams.mulUnroll == xLen
  val decode_table = {
    require(!usingRoCC || !rocketParams.useSCIE)
    (if (usingMulDiv) new MDecode(pipelinedMul, aluFn) +: (xLen > 32).option(new M64Decode(pipelinedMul, aluFn)).toSeq else Nil) ++:
    (if (usingAtomics) new ADecode(aluFn) +: (xLen > 32).option(new A64Decode(aluFn)).toSeq else Nil) ++:
    (if (fLen >= 32)    new FDecode(aluFn) +: (xLen > 32).option(new F64Decode(aluFn)).toSeq else Nil) ++:
    (if (fLen >= 64)    new DDecode(aluFn) +: (xLen > 32).option(new D64Decode(aluFn)).toSeq else Nil) ++:
    (if (minFLen == 16) new HDecode(aluFn) +: (xLen > 32).option(new H64Decode(aluFn)).toSeq ++: (fLen >= 64).option(new HDDecode(aluFn)).toSeq else Nil) ++:
    (usingRoCC.option(new RoCCDecode(aluFn))) ++:
    (rocketParams.useSCIE.option(new SCIEDecode(aluFn))) ++:
    (if (usingBitManip) new ZBADecode +: (xLen == 64).option(new ZBA64Decode).toSeq ++: new ZBBMDecode +: new ZBBORCBDecode +: new ZBCRDecode +: new ZBSDecode +: (xLen == 32).option(new ZBS32Decode).toSeq ++: (xLen == 64).option(new ZBS64Decode).toSeq ++: new ZBBSEDecode +: new ZBBCDecode +: (xLen == 64).option(new ZBBC64Decode).toSeq else Nil) ++:
    (if (usingBitManip && !usingBitManipCrypto) (xLen == 32).option(new ZBBZE32Decode).toSeq ++: (xLen == 64).option(new ZBBZE64Decode).toSeq else Nil) ++:
    (if (usingBitManip || usingBitManipCrypto) new ZBBNDecode +: new ZBCDecode +: new ZBBRDecode +: (xLen == 32).option(new ZBBR32Decode).toSeq ++: (xLen == 64).option(new ZBBR64Decode).toSeq ++: (xLen == 32).option(new ZBBREV832Decode).toSeq ++: (xLen == 64).option(new ZBBREV864Decode).toSeq else Nil) ++:
    (if (usingBitManipCrypto) new ZBKXDecode +: new ZBKBDecode +: (xLen == 32).option(new ZBKB32Decode).toSeq ++: (xLen == 64).option(new ZBKB64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) (xLen == 32).option(new ZKND32Decode).toSeq ++: (xLen == 64).option(new ZKND64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) (xLen == 32).option(new ZKNE32Decode).toSeq ++: (xLen == 64).option(new ZKNE64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) new ZKNHDecode +: (xLen == 32).option(new ZKNH32Decode).toSeq ++: (xLen == 64).option(new ZKNH64Decode).toSeq else Nil) ++:
    (usingCryptoSM.option(new ZKSDecode)) ++:
    (if (xLen == 32) new I32Decode(aluFn) else new I64Decode(aluFn)) +:
    (usingVM.option(new SVMDecode(aluFn))) ++:
    (usingSupervisor.option(new SDecode(aluFn))) ++:
    (usingHypervisor.option(new HypervisorDecode(aluFn))) ++:
    ((usingHypervisor && (xLen == 64)).option(new Hypervisor64Decode(aluFn))) ++:
    (usingDebug.option(new DebugDecode(aluFn))) ++:
    (usingNMI.option(new NMIDecode(aluFn))) ++:
    (usingConditionalZero.option(new ConditionalZeroDecode(aluFn))) ++:
    Seq(new FenceIDecode(tile.dcache.flushOnFenceI, aluFn)) ++:
    coreParams.haveCFlush.option(new CFlushDecode(tile.dcache.canSupportCFlushLine, aluFn)) ++:
    rocketParams.haveCease.option(new CeaseDecode(aluFn)) ++:
    Seq(new IDecode(aluFn))
  } flatMap(_.table)

  val ex_ctrl = Reg(new IntCtrlSigs(aluFn))
  val mem_ctrl = Reg(new IntCtrlSigs(aluFn))
  val wb_ctrl = Reg(new IntCtrlSigs(aluFn))

  val ex_reg_xcpt_interrupt  = Reg(Bool())
  val ex_reg_valid           = Reg(Bool())
  val ex_reg_rvc             = Reg(Bool())
  val ex_reg_btb_resp        = Reg(new BTBResp)
  val ex_reg_xcpt            = Reg(Bool())
  val ex_reg_flush_pipe      = Reg(Bool())
  val ex_reg_load_use        = Reg(Bool())
  val ex_reg_cause           = Reg(UInt())
  val ex_reg_replay = Reg(Bool())
  val ex_reg_pc = Reg(UInt())
  /*runahead code begin*/
  val ex_rh_store = Reg(Bool())
  val ex_rh_load = Reg(Bool())
  val ex_rh_hit = Wire(Bool())
  val ex_dmem_req_inv = Wire(Bool())
  val ex_hit_ptr = WireInit(0.U(5.W))
  /*runahead code end*/
  val ex_reg_mem_size = Reg(UInt())
  val ex_reg_hls = Reg(Bool())
  val ex_reg_inst = Reg(Bits())
  val ex_reg_raw_inst = Reg(UInt())
  val ex_scie_unpipelined = Reg(Bool())
  val ex_scie_pipelined = Reg(Bool())
  val ex_reg_wphit            = Reg(Vec(nBreakpoints, Bool()))

  val mem_reg_xcpt_interrupt  = Reg(Bool())
  val mem_reg_valid           = Reg(Bool())
  val mem_reg_rvc             = Reg(Bool())
  val mem_reg_btb_resp        = Reg(new BTBResp)
  val mem_reg_xcpt            = Reg(Bool())
  val mem_reg_replay          = Reg(Bool())
  val mem_reg_flush_pipe      = Reg(Bool())
  val mem_reg_cause           = Reg(UInt())
  val mem_reg_slow_bypass     = Reg(Bool())
  val mem_reg_load            = Reg(Bool())
  val mem_reg_store           = Reg(Bool())
  val mem_reg_sfence = Reg(Bool())
  val mem_reg_pc = Reg(UInt())
  /*runahead code begin*/
  val mem_dmem_req_inv = Reg(Bool())
  val mem_rh_load = Reg(Bool())
  val mem_rh_store = Reg(Bool())
  val mem_rh_hit = Reg(Bool())
//  val mem_rh_load_resp = Reg(Bool())
  val mem_hit_ptr = Reg(Bits())
  /*runahead code end*/
  val mem_reg_inst = Reg(Bits())
  val mem_reg_mem_size = Reg(UInt())
  val mem_reg_hls_or_dv = Reg(Bool())
  val mem_reg_raw_inst = Reg(UInt())
  val mem_scie_unpipelined = Reg(Bool())
  val mem_scie_pipelined = Reg(Bool())
  val mem_reg_wdata = Reg(Bits())
  val mem_reg_rs2 = Reg(Bits())
  val mem_br_taken = Reg(Bool())
  val take_pc_mem = Wire(Bool())
  val mem_reg_wphit          = Reg(Vec(nBreakpoints, Bool()))

  val wb_reg_valid           = Reg(Bool())
  val wb_reg_xcpt            = Reg(Bool())
  val wb_reg_replay          = Reg(Bool())
  val wb_reg_flush_pipe      = Reg(Bool())
  val wb_reg_cause           = Reg(UInt())
  val wb_reg_sfence = Reg(Bool())
  val wb_reg_pc = Reg(UInt())
  /*runahead code begin*/
  val wb_dmem_req_inv = Reg(Bool())
  val wb_reg_store = Reg(Bool())
  val wb_reg_load = Reg(Bool())
  val wb_rh_hit = Reg(Bool())
  val runahead_posedge = Wire(Bool())
  val s1_runahead_posedge = RegNext(runahead_posedge, init = false.B)
  val s2_runahead_posedge = RegNext(s1_runahead_posedge, init = false.B)
  val db_flag = Reg(Bool())
  val s1_db_flag = RegNext(db_flag,init = false.B)
  val s2_db_flag = RegNext(s1_db_flag,init = false.B)
  val s3_db_flag = RegNext(s2_db_flag,init = false.B)
  val exit_miss_back = Wire(Bool())
//  val wb_rh_load_resp = Reg(Bool())
  val wb_hit_ptr = Reg(Bits())
  /*runahead code end*/
  val wb_reg_mem_size = Reg(UInt())
  val wb_reg_hls_or_dv = Reg(Bool())
  val wb_reg_hfence_v = Reg(Bool())
  val wb_reg_hfence_g = Reg(Bool())
  val wb_reg_inst = Reg(Bits())
  val wb_reg_raw_inst = Reg(UInt())
  val wb_reg_wdata = Reg(Bits())
  val wb_reg_rs2 = Reg(Bits())
  val take_pc_wb = Wire(Bool())
  val wb_reg_wphit           = Reg(Vec(nBreakpoints, Bool()))

  val take_pc_mem_wb = take_pc_wb || take_pc_mem || db_flag //|| exit_miss_back
  val take_pc = take_pc_mem_wb &&
  /*runahead code begin*/
      !s1_db_flag && !s2_db_flag && !s3_db_flag

  val l2miss_tag = RegInit(0.U(7.W))
  val l2miss_addr = RegInit(0.U(40.W))
  val runahead_tag = RegInit(VecInit(Seq.fill(4)(0.U(7.W))))
  val runahead_addr = RegInit(VecInit(Seq.fill(4)(0.U(40.W))))
  val dcache_miss = RegInit(VecInit(Seq.fill(4)(false.B)))
  val s1_dcache_miss = RegNext(dcache_miss)
  val s2_dcache_miss = RegNext(s1_dcache_miss)
  val mshr_l2dcache_miss = RegInit(VecInit(Seq.fill(4)(false.B)))
  val l2cache_miss = RegInit(false.B)
  val runahead_enq_ptr_value = RegInit(VecInit(Seq.fill(4)(0.U(4.W))))
  val runahead_deq_ptr_value = RegInit(VecInit(Seq.fill(4)(0.U(4.W))))
  val s1_mshr_state = RegNext(io.dmem.mshr_state)
  val s1_mshr_enq_ptr_value = RegInit(VecInit(Seq.fill(4)(0.U(4.W))))
  val s1_mshr_deq_ptr_value = RegInit(VecInit(Seq.fill(4)(0.U(4.W))))
  s1_mshr_enq_ptr_value := io.dmem.mshr_enq_ptr_value
  s1_mshr_deq_ptr_value := io.dmem.mshr_deq_ptr_value
  val runahead_l2miss_events = RegInit(VecInit(Seq.fill(4)(false.B)))
  val block_addr = Wire(Vec(4, UInt(34.W)))
  val rfinvfile_block_miss = Wire(Vec(4,Bool()))

  val rcu = Module(new RCU(RCU_Params(xLen)))
  val runahead_flag = Wire(Bool())
  val dmem_req_addr = Wire(Bits())
  val s0_db_flag = Wire(Bool())
  rfinvfile_block_miss(0) :=  runahead_flag && dmem_req_addr(39,6) === block_addr(0) && ex_ctrl.mem && ex_ctrl.mem_cmd === M_XRD
  rfinvfile_block_miss(1) :=  runahead_flag && dmem_req_addr(39,6) === block_addr(1) && ex_ctrl.mem && ex_ctrl.mem_cmd === M_XRD && runahead_l2miss_events(1)
  rfinvfile_block_miss(2) :=  runahead_flag && dmem_req_addr(39,6) === block_addr(2) && ex_ctrl.mem && ex_ctrl.mem_cmd === M_XRD && runahead_l2miss_events(2)
  rfinvfile_block_miss(3) :=  runahead_flag && dmem_req_addr(39,6) === block_addr(3) && ex_ctrl.mem && ex_ctrl.mem_cmd === M_XRD && runahead_l2miss_events(3)
  //dontTouch(take_pc)
  dontTouch(rfinvfile_block_miss)
  dontTouch(block_addr)
  dontTouch(runahead_flag)
  /*runahead code end*/


  //========================================================= decode stage ==========================================================================
  val ibuf = Module(new IBuf)
  val id_expanded_inst = ibuf.io.inst.map(_.bits.inst)
  val id_raw_inst = ibuf.io.inst.map(_.bits.raw)
  val id_inst = id_expanded_inst.map(_.bits)
  ibuf.io.imem <> io.imem.resp
  ibuf.io.kill := take_pc
  /*runahead code begin*/
  rcu.io.ipc := ibuf.io.pc //wb_reg_pc
  rcu.io.wb_valid := s2_db_flag
  /*runahead code end*/

  require(decodeWidth == 1 /* TODO */ && retireWidth == decodeWidth)
  require(!(coreParams.useRVE && coreParams.fpu.nonEmpty), "Can't select both RVE and floating-point")
  require(!(coreParams.useRVE && coreParams.useHypervisor), "Can't select both RVE and Hypervisor")
  val id_ctrl = Wire(new IntCtrlSigs(aluFn)).decode(id_inst(0), decode_table)
  val lgNXRegs = if (coreParams.useRVE) 4 else 5
  val regAddrMask = (1 << lgNXRegs) - 1

  def decodeReg(x: UInt) = (x.extract(x.getWidth-1, lgNXRegs).asBool, x(lgNXRegs-1, 0))
  val (id_raddr3_illegal, id_raddr3) = decodeReg(id_expanded_inst(0).rs3)
  val (id_raddr2_illegal, id_raddr2) = decodeReg(id_expanded_inst(0).rs2)
  val (id_raddr1_illegal, id_raddr1) = decodeReg(id_expanded_inst(0).rs1)
  val (id_waddr_illegal,  id_waddr)  = decodeReg(id_expanded_inst(0).rd)

  val id_load_use = Wire(Bool())
  val id_reg_fence = RegInit(false.B)
  val id_ren = IndexedSeq(id_ctrl.rxs1, id_ctrl.rxs2)
  val id_raddr = IndexedSeq(id_raddr1, id_raddr2)
  val rf = new RegFile(regAddrMask, xLen)
  val id_rs = id_raddr.map(rf.read _)
  val ctrl_killd = Wire(Bool())
  val id_npc = (ibuf.io.pc.asSInt + ImmGen(IMM_UJ, id_inst(0))).asUInt

  val csr = Module(new CSRFile(perfEvents, coreParams.customCSRs.decls, tile.roccCSRs.flatten))
  val id_csr_en = id_ctrl.csr.isOneOf(CSR.S, CSR.C, CSR.W)
  val id_system_insn = id_ctrl.csr === CSR.I
  val id_csr_ren = id_ctrl.csr.isOneOf(CSR.S, CSR.C) && id_expanded_inst(0).rs1 === 0.U
  val id_csr = Mux(id_system_insn && id_ctrl.mem, CSR.N, Mux(id_csr_ren, CSR.R, id_ctrl.csr))
  val id_csr_flush = id_system_insn || (id_csr_en && !id_csr_ren && csr.io.decode(0).write_flush)

  val id_scie_decoder = if (!rocketParams.useSCIE) WireDefault(0.U.asTypeOf(new SCIEDecoderInterface)) else {
    val d = Module(new SCIEDecoder)
    assert(!io.imem.resp.valid || PopCount(d.io.unpipelined :: d.io.pipelined :: d.io.multicycle :: Nil) <= 1.U)
    d.io.insn := id_raw_inst(0)
    d.io
  }
  val id_illegal_rnum = if (usingCryptoNIST) (id_ctrl.zkn && aluFn.isKs1(id_ctrl.alu_fn) && id_inst(0)(23,20) > 0xA.U(4.W)) else false.B
  val id_illegal_insn = !id_ctrl.legal ||
    (id_ctrl.mul || id_ctrl.div) && !csr.io.status.isa('m'-'a') ||
    id_ctrl.amo && !csr.io.status.isa('a'-'a') ||
    id_ctrl.fp && (csr.io.decode(0).fp_illegal || io.fpu.illegal_rm) ||
    id_ctrl.dp && !csr.io.status.isa('d'-'a') ||
    ibuf.io.inst(0).bits.rvc && !csr.io.status.isa('c'-'a') ||
    id_raddr2_illegal && !id_ctrl.scie && id_ctrl.rxs2 ||
    id_raddr1_illegal && !id_ctrl.scie && id_ctrl.rxs1 ||
    id_waddr_illegal && !id_ctrl.scie && id_ctrl.wxd ||
    id_ctrl.rocc && csr.io.decode(0).rocc_illegal ||
    id_ctrl.scie && !(id_scie_decoder.unpipelined || id_scie_decoder.pipelined) ||
    id_csr_en && (csr.io.decode(0).read_illegal || !id_csr_ren && csr.io.decode(0).write_illegal) ||
    !ibuf.io.inst(0).bits.rvc && (id_system_insn && csr.io.decode(0).system_illegal) ||
    id_illegal_rnum
  val id_virtual_insn = id_ctrl.legal &&
    ((id_csr_en && !(!id_csr_ren && csr.io.decode(0).write_illegal) && csr.io.decode(0).virtual_access_illegal) ||
     (!ibuf.io.inst(0).bits.rvc && id_system_insn && csr.io.decode(0).virtual_system_illegal))
  // stall decode for fences (now, for AMO.rl; later, for AMO.aq and FENCE)
  val id_amo_aq = id_inst(0)(26)
  val id_amo_rl = id_inst(0)(25)
  val id_fence_pred = id_inst(0)(27,24)
  val id_fence_succ = id_inst(0)(23,20)
  val id_fence_next = id_ctrl.fence || id_ctrl.amo && id_amo_aq
  val id_mem_busy = !io.dmem.ordered || io.dmem.req.valid
  when (!id_mem_busy) { id_reg_fence := false.B }
  val id_rocc_busy = usingRoCC.B &&
    (io.rocc.busy || ex_reg_valid && ex_ctrl.rocc ||
     mem_reg_valid && mem_ctrl.rocc || wb_reg_valid && wb_ctrl.rocc)
  val id_do_fence = WireDefault(id_rocc_busy && id_ctrl.fence ||
    id_mem_busy && (id_ctrl.amo && id_amo_rl || id_ctrl.fence_i || id_reg_fence && (id_ctrl.mem || id_ctrl.rocc)))

  val bpu = Module(new BreakpointUnit(nBreakpoints))
  bpu.io.status := csr.io.status
  bpu.io.bp := csr.io.bp
  bpu.io.pc := ibuf.io.pc
  bpu.io.ea := mem_reg_wdata
  bpu.io.mcontext := csr.io.mcontext
  bpu.io.scontext := csr.io.scontext

  val id_xcpt0 = ibuf.io.inst(0).bits.xcpt0
  val id_xcpt1 = ibuf.io.inst(0).bits.xcpt1
  val (id_xcpt, id_cause) = checkExceptions(List(
    (csr.io.interrupt, csr.io.interrupt_cause),
    (bpu.io.debug_if,  CSR.debugTriggerCause.U),
    (bpu.io.xcpt_if,   Causes.breakpoint.U),
    (id_xcpt0.pf.inst, Causes.fetch_page_fault.U),
    (id_xcpt0.gf.inst, Causes.fetch_guest_page_fault.U),
    (id_xcpt0.ae.inst, Causes.fetch_access.U),
    (id_xcpt1.pf.inst, Causes.fetch_page_fault.U),
    (id_xcpt1.gf.inst, Causes.fetch_guest_page_fault.U),
    (id_xcpt1.ae.inst, Causes.fetch_access.U),
    (id_virtual_insn,  Causes.virtual_instruction.U),
    (id_illegal_insn,  Causes.illegal_instruction.U)))

  val idCoverCauses = List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.fetch_access, "FETCH_ACCESS"),
    (Causes.illegal_instruction, "ILLEGAL_INSTRUCTION")
  ) ++ (if (usingVM) List(
    (Causes.fetch_page_fault, "FETCH_PAGE_FAULT")
  ) else Nil)
  coverExceptions(id_xcpt, id_cause, "DECODE", idCoverCauses)

  val dcache_bypass_data =
    if (fastLoadByte) io.dmem.resp.bits.data(xLen-1, 0)
    else if (fastLoadWord) io.dmem.resp.bits.data_word_bypass(xLen-1, 0)
    else wb_reg_wdata

  // detect bypass opportunities
  val ex_waddr = ex_reg_inst(11,7) & regAddrMask.U
  val mem_waddr = mem_reg_inst(11,7) & regAddrMask.U
  val wb_waddr = wb_reg_inst(11,7) & regAddrMask.U
  val bypass_sources = IndexedSeq(
    (true.B, 0.U, 0.U), // treat reading x0 as a bypass
    (ex_reg_valid && ex_ctrl.wxd, ex_waddr, mem_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd && !mem_ctrl.mem, mem_waddr, wb_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd, mem_waddr, dcache_bypass_data))
  val id_bypass_src = id_raddr.map(raddr => bypass_sources.map(s => s._1 && s._2 === raddr))

  //============================================================ execute stage ======================================================================================
  val bypass_mux = bypass_sources.map(_._3)
  val ex_reg_rs_bypass = Reg(Vec(id_raddr.size, Bool()))
  val ex_reg_rs_lsb = Reg(Vec(id_raddr.size, UInt(log2Ceil(bypass_sources.size).W)))
  val ex_reg_rs_msb = Reg(Vec(id_raddr.size, UInt()))
  val ex_rs = for (i <- 0 until id_raddr.size)
    yield Mux(ex_reg_rs_bypass(i), bypass_mux(ex_reg_rs_lsb(i)), Cat(ex_reg_rs_msb(i), ex_reg_rs_lsb(i)))
  val ex_imm = ImmGen(ex_ctrl.sel_imm, ex_reg_inst)
  val ex_op1 = MuxLookup(ex_ctrl.sel_alu1, 0.S, Seq(
    A1_RS1 -> ex_rs(0).asSInt,
    A1_PC -> ex_reg_pc.asSInt))
  val ex_op2 = MuxLookup(ex_ctrl.sel_alu2, 0.S, Seq(
    A2_RS2 -> ex_rs(1).asSInt,
    A2_IMM -> ex_imm,
    A2_SIZE -> Mux(ex_reg_rvc, 2.S, 4.S)))

  val alu = Module(aluFn match {
    case _: ABLUFN => new ABLU
    case _: ALUFN => new ALU
  })
  alu.io.dw := ex_ctrl.alu_dw
  alu.io.fn := ex_ctrl.alu_fn
  alu.io.in2 := ex_op2.asUInt
  alu.io.in1 := ex_op1.asUInt

  val ex_scie_unpipelined_wdata = if (!rocketParams.useSCIE) 0.U else {
    val u = Module(new SCIEUnpipelined(xLen))
    u.io.insn := ex_reg_inst
    u.io.rs1 := ex_rs(0)
    u.io.rs2 := ex_rs(1)
    u.io.rd
  }

  val mem_scie_pipelined_wdata = if (!rocketParams.useSCIE) 0.U else {
    val u = Module(new SCIEPipelined(xLen))
    u.io.clock := Module.clock
    u.io.valid := ex_reg_valid && ex_scie_pipelined
    u.io.insn := ex_reg_inst
    u.io.rs1 := ex_rs(0)
    u.io.rs2 := ex_rs(1)
    u.io.rd
  }

  val ex_zbk_wdata = if (!usingBitManipCrypto && !usingBitManip) 0.U else {
    val zbk = Module(new BitManipCrypto(xLen))
    zbk.io.fn  := ex_ctrl.alu_fn
    zbk.io.dw  := ex_ctrl.alu_dw
    zbk.io.rs1 := ex_op1.asUInt
    zbk.io.rs2 := ex_op2.asUInt
    zbk.io.rd
  }

  val ex_zkn_wdata = if (!usingCryptoNIST) 0.U else {
    val zkn = Module(new CryptoNIST(xLen))
    zkn.io.fn   := ex_ctrl.alu_fn
    zkn.io.hl   := ex_reg_inst(27)
    zkn.io.bs   := ex_reg_inst(31,30)
    zkn.io.rs1  := ex_op1.asUInt
    zkn.io.rs2  := ex_op2.asUInt
    zkn.io.rd
  }

  val ex_zks_wdata = if (!usingCryptoSM) 0.U else {
    val zks = Module(new CryptoSM(xLen))
    zks.io.fn  := ex_ctrl.alu_fn
    zks.io.bs  := ex_reg_inst(31,30)
    zks.io.rs1 := ex_op1.asUInt
    zks.io.rs2 := ex_op2.asUInt
    zks.io.rd
  }

  // multiplier and divider
  val div = Module(new MulDiv(if (pipelinedMul) mulDivParams.copy(mulUnroll = 0) else mulDivParams, width = xLen, aluFn = aluFn))
  div.io.req.valid := ex_reg_valid && ex_ctrl.div
  div.io.req.bits.dw := ex_ctrl.alu_dw
  div.io.req.bits.fn := ex_ctrl.alu_fn
  div.io.req.bits.in1 := ex_rs(0)
  div.io.req.bits.in2 := ex_rs(1)
  div.io.req.bits.tag := ex_waddr
  val mul = pipelinedMul.option {
    val m = Module(new PipelinedMultiplier(xLen, 2, aluFn = aluFn))
    m.io.req.valid := ex_reg_valid && ex_ctrl.mul
    m.io.req.bits := div.io.req.bits
    m
  }

  ex_reg_valid := !ctrl_killd
  ex_reg_replay := !take_pc && ibuf.io.inst(0).valid && ibuf.io.inst(0).bits.replay
  ex_reg_xcpt := !ctrl_killd && id_xcpt
  ex_reg_xcpt_interrupt := !take_pc && ibuf.io.inst(0).valid && csr.io.interrupt

  when (!ctrl_killd) {
    ex_ctrl := id_ctrl
    ex_reg_rvc := ibuf.io.inst(0).bits.rvc
    ex_ctrl.csr := id_csr
    ex_scie_unpipelined := id_ctrl.scie && id_scie_decoder.unpipelined
    ex_scie_pipelined := id_ctrl.scie && id_scie_decoder.pipelined
    when (id_ctrl.fence && id_fence_succ === 0.U) { id_reg_pause := true.B }
    when (id_fence_next) { id_reg_fence := true.B }
    when (id_xcpt) { // pass PC down ALU writeback pipeline for badaddr
      ex_ctrl.alu_fn := aluFn.FN_ADD
      ex_ctrl.alu_dw := DW_XPR
      ex_ctrl.sel_alu1 := A1_RS1 // badaddr := instruction
      ex_ctrl.sel_alu2 := A2_ZERO
      when (id_xcpt1.asUInt.orR) { // badaddr := PC+2
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_SIZE
        ex_reg_rvc := true.B
      }
      when (bpu.io.xcpt_if || id_xcpt0.asUInt.orR) { // badaddr := PC
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_ZERO
      }
    }
    ex_reg_flush_pipe := id_ctrl.fence_i || id_csr_flush
    ex_reg_load_use := id_load_use
    ex_reg_hls := usingHypervisor.B && id_system_insn && id_ctrl.mem_cmd.isOneOf(M_XRD, M_XWR, M_HLVX)
    ex_reg_mem_size := Mux(usingHypervisor.B && id_system_insn, id_inst(0)(27, 26), id_inst(0)(13, 12))
    when (id_ctrl.mem_cmd.isOneOf(M_SFENCE, M_HFENCEV, M_HFENCEG, M_FLUSH_ALL)) {
      ex_reg_mem_size := Cat(id_raddr2 =/= 0.U, id_raddr1 =/= 0.U)
    }
    when (id_ctrl.mem_cmd === M_SFENCE && csr.io.status.v) {
      ex_ctrl.mem_cmd := M_HFENCEV
    }
    if (tile.dcache.flushOnFenceI) {
      when (id_ctrl.fence_i) {
        ex_reg_mem_size := 0.U
      }
    }

    for (i <- 0 until id_raddr.size) {
      val do_bypass = id_bypass_src(i).reduce(_||_)
      val bypass_src = PriorityEncoder(id_bypass_src(i))
      ex_reg_rs_bypass(i) := do_bypass
      ex_reg_rs_lsb(i) := bypass_src
      when (id_ren(i) && !do_bypass) {
        ex_reg_rs_lsb(i) := id_rs(i)(log2Ceil(bypass_sources.size)-1, 0)
        ex_reg_rs_msb(i) := id_rs(i) >> log2Ceil(bypass_sources.size)
      }
    }
    when (id_illegal_insn || id_virtual_insn) {
      val inst = Mux(ibuf.io.inst(0).bits.rvc, id_raw_inst(0)(15, 0), id_raw_inst(0))
      ex_reg_rs_bypass(0) := false.B
      ex_reg_rs_lsb(0) := inst(log2Ceil(bypass_sources.size)-1, 0)
      ex_reg_rs_msb(0) := inst >> log2Ceil(bypass_sources.size)
    }
  }
  when (!ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay) {
    ex_reg_cause := id_cause
    ex_reg_inst := id_inst(0)
    ex_reg_raw_inst := id_raw_inst(0)
    ex_reg_pc := ibuf.io.pc
    ex_reg_btb_resp := ibuf.io.btb_resp
    ex_reg_wphit := bpu.io.bpwatch.map { bpw => bpw.ivalid(0) }

    /*runahead code begin*/
    ex_rh_load := id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && (runahead_flag === true.B)
    ex_rh_store := id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && (runahead_flag === true.B)
    /*runahead code end*/


  }

  // replay inst in ex stage?
  val ex_pc_valid = ex_reg_valid || ex_reg_replay || ex_reg_xcpt_interrupt
  val wb_dcache_miss = wb_ctrl.mem && !io.dmem.resp.valid &&
  /*runahead code begin*/
  !wb_dmem_req_inv && !exit_miss_back
  /*runahead code end*/
  val replay_ex_structural = ex_ctrl.mem && !io.dmem.req.ready ||
                             ex_ctrl.div && !div.io.req.ready
  val replay_ex_load_use = wb_dcache_miss && ex_reg_load_use
  val replay_ex = ex_reg_replay || (ex_reg_valid && (replay_ex_structural || replay_ex_load_use))
  val ctrl_killx = take_pc_mem_wb || replay_ex || !ex_reg_valid
  // detect 2-cycle load-use delay for LB/LH/SC
  val ex_slow_bypass = ex_ctrl.mem_cmd === M_XSC || ex_reg_mem_size < 2.U
  val ex_sfence = usingVM.B && ex_ctrl.mem && (ex_ctrl.mem_cmd === M_SFENCE || ex_ctrl.mem_cmd === M_HFENCEV || ex_ctrl.mem_cmd === M_HFENCEG)

  //*****************dcache miss counter**********************************************
//   val counter = RegInit(0.U(32.W))
//   val miss_detected = RegInit(false.B)
//   val miss_events = RegInit(0.U(32.W))

//   when(reset.asBool()) {
//     counter := 0.U
//     miss_detected := false.B
//     miss_events := 0.U // 重置miss事件计数器
// }.otherwise {
//     when(wb_dcache_miss) {
//         // 检测到miss
//         when(!miss_detected) {
//             miss_detected := true.B
//         }
//         counter := counter + 1.U
//     }.elsewhen(miss_detected) {
//         // miss信号从高变低
//         when(counter > 2.U) {
//             // printf(p"DCache miss lasted for ${counter} cycles\n")
//             miss_events := miss_events + 40.U // 当miss的周期大于3时才增加miss事件计数
//         }
//         counter := 0.U
//         miss_detected := false.B
//     }
// }
// // printf(p"Total DCache miss events: ${miss_events}\n")


//   //dcache access count
//   val access_detected = RegInit(false.B)
//   val access_events = RegInit(0.U(32.W))

//     when(reset.asBool()) {    
//     access_detected := false.B
//     access_events := 0.U // 重置access事件计数器
// }.otherwise {
//     when(io.dmem.req.valid) {
//         // access count
//         when(!access_detected) {
//             access_detected := true.B
//             access_events := access_events + 1.U // 在首次检测到access时增加access事件计数
//         }
//     }.elsewhen(access_detected) {
//         // access信号从高变低，然后重置flag
//         // printf(p"DCache access detect!\n")
//         access_detected := false.B
//     }
// }
// printf(p"Total DCache access events: ${access_events}\n")
//********************************************************************************************
//记录所有fp inst numbers
//   val fp_events = RegInit(0.U(32.W))
//   val fp_eventsnext = RegInit(0.U(32.W))

// //   when(reset.asBool()) {
// //     fp_events := 0.U // 重置fp事件计数器
// // }.otherwise {
// //     when(ex_ctrl.fp && (ex_reg_inst =/= mem_reg_inst)) {
// //         // 检测到 fp
// //         fp_events := fp_events + 1.U
// //     }
// // }

// when (fp_events =/= fp_eventsnext) {
//   printf(p"Total fp events: ${fp_events}\n")
//   fp_eventsnext := fp_events
// }

//  val csr_ctrl = Wire(new IntCtrlSigs(aluFn)).decode(csr.io.trace(0).insn, decode_table)
//  when (csr.io.trace(0).valid){
//     when (csr_ctrl.fp) {
//       fp_events := fp_events + 1.U
//     }
//  }

  //*******************************************************************************************
  val (ex_xcpt, ex_cause) = checkExceptions(List(
    (ex_reg_xcpt_interrupt || ex_reg_xcpt, ex_reg_cause)))

  val exCoverCauses = idCoverCauses
  coverExceptions(ex_xcpt, ex_cause, "EXECUTE", exCoverCauses)

  //============================================= memory stage =======================================================================
  val mem_pc_valid = mem_reg_valid || mem_reg_replay || mem_reg_xcpt_interrupt
  val mem_br_target = mem_reg_pc.asSInt +
    Mux(mem_ctrl.branch && mem_br_taken, ImmGen(IMM_SB, mem_reg_inst),
    Mux(mem_ctrl.jal, ImmGen(IMM_UJ, mem_reg_inst),
    Mux(mem_reg_rvc, 2.S, 4.S)))
  val mem_npc = (Mux(mem_ctrl.jalr || mem_reg_sfence, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target) & (-2).S).asUInt
  val mem_wrong_npc =
    Mux(ex_pc_valid, mem_npc =/= ex_reg_pc,
    Mux(ibuf.io.inst(0).valid || ibuf.io.imem.valid, mem_npc =/= ibuf.io.pc, true.B))
  val mem_npc_misaligned = !csr.io.status.isa('c'-'a') && mem_npc(1) && !mem_reg_sfence
  val mem_int_wdata = Mux(!mem_reg_xcpt && (mem_ctrl.jalr ^ mem_npc_misaligned), mem_br_target, mem_reg_wdata.asSInt).asUInt
  val mem_cfi = mem_ctrl.branch || mem_ctrl.jalr || mem_ctrl.jal
  val mem_cfi_taken = (mem_ctrl.branch && mem_br_taken) || mem_ctrl.jalr || mem_ctrl.jal
  val mem_direction_misprediction = mem_ctrl.branch && mem_br_taken =/= (usingBTB.B && mem_reg_btb_resp.taken)
  val mem_misprediction = if (usingBTB) mem_wrong_npc else mem_cfi_taken
  take_pc_mem := mem_reg_valid && !mem_reg_xcpt && (mem_misprediction || mem_reg_sfence)

  mem_reg_valid := !ctrl_killx
  mem_reg_replay := !take_pc_mem_wb && replay_ex
  mem_reg_xcpt := !ctrl_killx && ex_xcpt
  mem_reg_xcpt_interrupt := !take_pc_mem_wb && ex_reg_xcpt_interrupt

  // on pipeline flushes, cause mem_npc to hold the sequential npc, which
  // will drive the W-stage npc mux
  when (mem_reg_valid && mem_reg_flush_pipe) {
    mem_reg_sfence := false.B
  }.elsewhen (ex_pc_valid) {
    mem_ctrl := ex_ctrl
    mem_scie_unpipelined := ex_scie_unpipelined
    mem_scie_pipelined := ex_scie_pipelined
    mem_reg_rvc := ex_reg_rvc
    mem_reg_load := ex_ctrl.mem && isRead(ex_ctrl.mem_cmd)
    mem_reg_store := ex_ctrl.mem && isWrite(ex_ctrl.mem_cmd)
    /*runahead code begin*/
    mem_rh_load := ex_rh_load
    mem_rh_store := ex_rh_store
    mem_dmem_req_inv := ex_dmem_req_inv
    mem_rh_hit := ex_rh_hit
    mem_hit_ptr := ex_hit_ptr
    // when((ex_rh_load === true.B) && (runahead_flag === true.B) && (ex_rh_hit === true.B)){//load the data
    //   mem_rh_load_resp := true.B
    // }.otherwise{
    //   mem_rh_load_resp := false.B
    // }
    /*runahead code end*/
    mem_reg_sfence := ex_sfence
    mem_reg_btb_resp := ex_reg_btb_resp
    mem_reg_flush_pipe := ex_reg_flush_pipe
    mem_reg_slow_bypass := ex_slow_bypass
    mem_reg_wphit := ex_reg_wphit

    mem_reg_cause := ex_cause
    mem_reg_inst := ex_reg_inst
    mem_reg_raw_inst := ex_reg_raw_inst
    mem_reg_mem_size := ex_reg_mem_size
    mem_reg_hls_or_dv := io.dmem.req.bits.dv
    mem_reg_pc := ex_reg_pc
    // IDecode ensured they are 1H
    mem_reg_wdata := Mux1H(Seq(
      ex_scie_unpipelined -> ex_scie_unpipelined_wdata,
      ex_ctrl.zbk         -> ex_zbk_wdata,
      ex_ctrl.zkn         -> ex_zkn_wdata,
      ex_ctrl.zks         -> ex_zks_wdata,
      (!ex_scie_unpipelined && !ex_ctrl.zbk && !ex_ctrl.zkn && !ex_ctrl.zks)
                          -> alu.io.out,
    ))
    mem_br_taken := alu.io.cmp_out

    when (ex_ctrl.rxs2 && (ex_ctrl.mem || ex_ctrl.rocc || ex_sfence)) {
      val size = Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_reg_mem_size)
      mem_reg_rs2 := new StoreGen(size, 0.U, ex_rs(1), coreDataBytes).data
    }
    when (ex_ctrl.jalr && csr.io.status.debug) {
      // flush I$ on D-mode JALR to effect uncached fetch without D$ flush
      mem_ctrl.fence_i := true.B
      mem_reg_flush_pipe := true.B
    }
  }

  val mem_breakpoint = (mem_reg_load && bpu.io.xcpt_ld) || (mem_reg_store && bpu.io.xcpt_st)
  val mem_debug_breakpoint = (mem_reg_load && bpu.io.debug_ld) || (mem_reg_store && bpu.io.debug_st)
  val (mem_ldst_xcpt, mem_ldst_cause) = checkExceptions(List(
    (mem_debug_breakpoint, CSR.debugTriggerCause.U),
    (mem_breakpoint,       Causes.breakpoint.U)))

  val (mem_xcpt, mem_cause) = checkExceptions(List(
    (mem_reg_xcpt_interrupt || mem_reg_xcpt, mem_reg_cause),
    (mem_reg_valid && mem_npc_misaligned,    Causes.misaligned_fetch.U),
    (mem_reg_valid && mem_ldst_xcpt,         mem_ldst_cause)))

  val memCoverCauses = (exCoverCauses ++ List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.misaligned_fetch, "MISALIGNED_FETCH")
  )).distinct
  coverExceptions(mem_xcpt, mem_cause, "MEMORY", memCoverCauses)

  val dcache_kill_mem = mem_reg_valid && mem_ctrl.wxd && io.dmem.replay_next // structural hazard on writeback port
  val fpu_kill_mem = mem_reg_valid && mem_ctrl.fp && io.fpu.nack_mem
  val replay_mem  = dcache_kill_mem || mem_reg_replay || fpu_kill_mem
  val killm_common = dcache_kill_mem || take_pc_wb || mem_reg_xcpt || !mem_reg_valid 
  div.io.kill := killm_common && RegNext(div.io.req.fire) ||
  /*runahead code begin*/
  s0_db_flag
  /*runahead code end*/
  val ctrl_killm = killm_common || mem_xcpt || fpu_kill_mem 

  // =================================================writeback stage=================================================================
  wb_reg_valid := !ctrl_killm
  wb_reg_replay := replay_mem && !take_pc_wb
  wb_reg_xcpt := mem_xcpt && !take_pc_wb
  wb_reg_flush_pipe := !ctrl_killm && mem_reg_flush_pipe
  when (mem_pc_valid) {
    wb_ctrl := mem_ctrl
    wb_reg_sfence := mem_reg_sfence
    wb_reg_wdata := Mux(mem_scie_pipelined, mem_scie_pipelined_wdata,
      Mux(!mem_reg_xcpt && mem_ctrl.fp && mem_ctrl.wxd, io.fpu.toint_data, mem_int_wdata))
    when (mem_ctrl.rocc || mem_reg_sfence) {
      wb_reg_rs2 := mem_reg_rs2
    }
    wb_reg_cause := mem_cause
    wb_reg_inst := mem_reg_inst
    wb_reg_raw_inst := mem_reg_raw_inst
    wb_reg_mem_size := mem_reg_mem_size
    wb_reg_hls_or_dv := mem_reg_hls_or_dv
    wb_reg_hfence_v := mem_ctrl.mem_cmd === M_HFENCEV
    wb_reg_hfence_g := mem_ctrl.mem_cmd === M_HFENCEG
    wb_reg_pc := mem_reg_pc
    /*runahead code begin*/
    wb_reg_store := mem_reg_store
    wb_reg_load := mem_reg_load
    wb_dmem_req_inv := mem_dmem_req_inv
    wb_rh_hit := mem_rh_hit
//    wb_rh_load_resp := mem_rh_load_resp
    wb_hit_ptr := mem_hit_ptr
    /*runahead code end*/

    wb_reg_wphit := mem_reg_wphit | bpu.io.bpwatch.map { bpw => (bpw.rvalid(0) && mem_reg_load) || (bpw.wvalid(0) && mem_reg_store) }
  }

  val (wb_xcpt, wb_cause) = checkExceptions(List(
    (wb_reg_xcpt,  wb_reg_cause),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.pf.st, Causes.store_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.pf.ld, Causes.load_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.gf.st, Causes.store_guest_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.gf.ld, Causes.load_guest_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ae.st, Causes.store_access.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ae.ld, Causes.load_access.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ma.st, Causes.misaligned_store.U),
    (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ma.ld, Causes.misaligned_load.U)
  ))

  val wbCoverCauses = List(
    (Causes.misaligned_store, "MISALIGNED_STORE"),
    (Causes.misaligned_load, "MISALIGNED_LOAD"),
    (Causes.store_access, "STORE_ACCESS"),
    (Causes.load_access, "LOAD_ACCESS")
  ) ++ (if(usingVM) List(
    (Causes.store_page_fault, "STORE_PAGE_FAULT"),
    (Causes.load_page_fault, "LOAD_PAGE_FAULT")
  ) else Nil) ++ (if (usingHypervisor) List(
    (Causes.store_guest_page_fault, "STORE_GUEST_PAGE_FAULT"),
    (Causes.load_guest_page_fault, "LOAD_GUEST_PAGE_FAULT"),
  ) else Nil)
  coverExceptions(wb_xcpt, wb_cause, "WRITEBACK", wbCoverCauses)

  val wb_pc_valid = wb_reg_valid || wb_reg_replay || wb_reg_xcpt
  val wb_wxd = wb_reg_valid && wb_ctrl.wxd
  val wb_set_sboard = wb_ctrl.div || wb_dcache_miss || wb_ctrl.rocc
  val replay_wb_common = io.dmem.s2_nack || wb_reg_replay
  val replay_wb_rocc = wb_reg_valid && wb_ctrl.rocc && !io.rocc.cmd.ready
  val replay_wb = (replay_wb_common || replay_wb_rocc) && 
  /*runahead code begin*/
  !s0_db_flag && !db_flag && !s1_db_flag
  take_pc_wb := replay_wb || wb_xcpt || csr.io.eret || wb_reg_flush_pipe

  // writeback arbitration
  val dmem_resp_xpu = !io.dmem.resp.bits.tag(0).asBool //||
  /*runahead code begin*/ 
  //(wb_rh_load_resp === true.B)
  /*runahead code end*/
  val dmem_resp_fpu =  io.dmem.resp.bits.tag(0).asBool
  /*runahead code begin*/
  //val dmem_resp_waddr = io.dmem.resp.bits.tag(5, 1)
  val dmem_resp_waddr = Mux(runahead_flag && wb_rh_hit && !s0_db_flag, wb_waddr, io.dmem.resp.bits.tag(5, 1))
  /*runahead code end*/
  val dmem_resp_valid = io.dmem.resp.valid && io.dmem.resp.bits.has_data && 
  !exit_miss_back
  /*runahead code end*/
  val dmem_resp_replay = dmem_resp_valid && io.dmem.resp.bits.replay

 /*runahead code begin*/
  io.imem.l2miss := rcu.io.l2miss
  io.imem.l2back := s2_db_flag
  io.imem.exit_miss_back := exit_miss_back
  val mshr_l2miss_tag = Mux(io.dmem.mshr_state(1) === 0.U,io.dmem.mshr_tag(0), 0.U)
  val mshr_l2miss_addr = Mux(io.dmem.mshr_state(1) === 0.U,io.dmem.mshr_addr(0), 0.U)
  val s_runahead_wait_req ::s_runahead_wait_resp :: s_runahead :: s_pass :: s_inv :: s_pseudo_exit :: s_exit :: Nil = Enum(7)
  val runahead_state = RegInit(s_runahead_wait_req)
  val s1_runahead = RegNext(runahead_state === s_runahead, init = false.B)
  val s2_runahead = RegNext(s1_runahead,init = false.B)
  runahead_posedge := Mux(runahead_state === s_runahead,!s2_runahead & s1_runahead,0.U)

 /*runahead code end*/

  div.io.resp.ready := !wb_wxd
  val ll_wdata = WireDefault(div.io.resp.bits.data)
  val ll_waddr = WireDefault(div.io.resp.bits.tag)
  val ll_wen = WireDefault(div.io.resp.fire)
  if (usingRoCC) {
    io.rocc.resp.ready := !wb_wxd
    when (io.rocc.resp.fire) {
      div.io.resp.ready := false.B
      ll_wdata := io.rocc.resp.bits.data
      ll_waddr := io.rocc.resp.bits.rd
      ll_wen := true.B
    }
  }
  when (dmem_resp_replay && dmem_resp_xpu) {
    div.io.resp.ready := false.B
    if (usingRoCC)
      io.rocc.resp.ready := false.B
    ll_waddr := dmem_resp_waddr
    ll_wen := true.B
  }

  val wb_valid = wb_reg_valid && !replay_wb && !wb_xcpt
  val wb_wen = wb_valid && wb_ctrl.wxd
  val rf_wen = (wb_wen || ll_wen) &&
  /*runahead code begin*/
  !(db_flag || s1_db_flag)  //在db_flag和之后的一个cycle不能写回

  val rhbuffer_data = RegInit(VecInit(Seq.fill(10)(0.U(64.W))))


  /*runahead code end*/
  val rf_waddr = Mux(ll_wen, ll_waddr, wb_waddr)
  val rf_wdata = 
                /*runahead code begin*/
                 Mux(runahead_flag && wb_rh_hit && !s0_db_flag, rhbuffer_data(wb_hit_ptr),
                /*runahead code end*/
                 Mux(dmem_resp_valid && dmem_resp_xpu, io.dmem.resp.bits.data(xLen-1, 0),
                 Mux(ll_wen, ll_wdata,
                 Mux(wb_ctrl.csr =/= CSR.N, csr.io.rw.rdata,
                 Mux(wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata),
                 wb_reg_wdata)))))
  /*runahead code begin*/
  for (i <- 0 until 31) {
    rcu.io.rf_in(i) := rf.read(i.U)
  }
  when(s2_db_flag) {
    for(j <- 0 until 31) {
      when(j.U =/= l2miss_tag(6, 2)){
        rf.write(j.U, rcu.io.rf_out(j))
      }
    }
  }
  exit_miss_back :=  (runahead_state === s_runahead || runahead_state === s_pseudo_exit) && io.dmem.resp.valid && (
                       io.dmem.resp.bits.addr(39,6) === runahead_addr(0)(39,6) && runahead_l2miss_events(0) === true.B ||
                       io.dmem.resp.bits.addr(39,6) === runahead_addr(1)(39,6) && runahead_l2miss_events(1) === true.B ||
                       io.dmem.resp.bits.addr(39,6) === runahead_addr(2)(39,6) && runahead_l2miss_events(2) === true.B ||
                       io.dmem.resp.bits.addr(39,6) === runahead_addr(3)(39,6) && runahead_l2miss_events(3) === true.B)
  /*runahead code end*/

  when (rf_wen //&&
  /*runahead code begin*/
 // !exit_miss_back
  /*runahead code end*/
  ) { rf.write(rf_waddr, rf_wdata) }

  // hook up control/status regfile
  csr.io.ungated_clock := clock
  csr.io.decode(0).inst := id_inst(0)
  csr.io.exception := wb_xcpt
  csr.io.cause := wb_cause
  csr.io.retire := wb_valid
  csr.io.inst(0) := (if (usingCompressed) Cat(Mux(wb_reg_raw_inst(1, 0).andR, wb_reg_inst >> 16, 0.U), wb_reg_raw_inst(15, 0)) else wb_reg_inst)
  csr.io.interrupts := io.interrupts
  csr.io.hartid := io.hartid
  io.fpu.fcsr_rm := csr.io.fcsr_rm
  csr.io.fcsr_flags := io.fpu.fcsr_flags
  io.fpu.time := csr.io.time(31,0)
  io.fpu.hartid := io.hartid
  csr.io.rocc_interrupt := io.rocc.interrupt
  csr.io.pc := wb_reg_pc
  val tval_dmem_addr = !wb_reg_xcpt
  val tval_any_addr = tval_dmem_addr ||
    wb_reg_cause.isOneOf(Causes.breakpoint.U, Causes.fetch_access.U, Causes.fetch_page_fault.U, Causes.fetch_guest_page_fault.U)
  val tval_inst = wb_reg_cause === Causes.illegal_instruction.U
  val tval_valid = wb_xcpt && (tval_any_addr || tval_inst)
  csr.io.gva := wb_xcpt && (tval_any_addr && csr.io.status.v || tval_dmem_addr && wb_reg_hls_or_dv)
  csr.io.tval := Mux(tval_valid, encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata), 0.U)
  csr.io.htval := {
    val htval_valid_imem = wb_reg_xcpt && wb_reg_cause === Causes.fetch_guest_page_fault.U
    val htval_imem = Mux(htval_valid_imem, io.imem.gpa.bits, 0.U)
    assert(!htval_valid_imem || io.imem.gpa.valid)

    val htval_valid_dmem = wb_xcpt && tval_dmem_addr && io.dmem.s2_xcpt.gf.asUInt.orR && !io.dmem.s2_xcpt.pf.asUInt.orR
    val htval_dmem = Mux(htval_valid_dmem, io.dmem.s2_gpa, 0.U)

    (htval_dmem | htval_imem) >> hypervisorExtraAddrBits
  }
  io.ptw.ptbr := csr.io.ptbr
  io.ptw.hgatp := csr.io.hgatp
  io.ptw.vsatp := csr.io.vsatp
  (io.ptw.customCSRs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs := rhs }
  io.ptw.status := csr.io.status
  io.ptw.hstatus := csr.io.hstatus
  io.ptw.gstatus := csr.io.gstatus
  io.ptw.pmp := csr.io.pmp
  csr.io.rw.addr := wb_reg_inst(31,20)
  csr.io.rw.cmd := CSR.maskCmd(wb_reg_valid, wb_ctrl.csr)
  csr.io.rw.wdata := wb_reg_wdata
  io.trace.insns := csr.io.trace
  io.trace.time := csr.io.time
  io.rocc.csrs := csr.io.roccCSRs
  if (rocketParams.debugROB) {
    val csr_trace_with_wdata = WireInit(csr.io.trace(0))
    csr_trace_with_wdata.wdata.get := rf_wdata
    DebugROB.pushTrace(clock, reset,
      io.hartid, csr_trace_with_wdata,
      (wb_ctrl.wfd || (wb_ctrl.wxd && wb_waddr =/= 0.U)) && !csr.io.trace(0).exception,
      wb_ctrl.wxd && wb_wen && !wb_set_sboard,
      wb_waddr + Mux(wb_ctrl.wfd, 32.U, 0.U))

    io.trace.insns(0) := DebugROB.popTrace(clock, reset, io.hartid)

    DebugROB.pushWb(clock, reset, io.hartid, ll_wen, rf_waddr, rf_wdata)
  }

  for (((iobpw, wphit), bp) <- io.bpwatch zip wb_reg_wphit zip csr.io.bp) {
    iobpw.valid(0) := wphit
    iobpw.action := bp.control.action
  }

  val hazard_targets = Seq((id_ctrl.rxs1 && id_raddr1 =/= 0.U, id_raddr1),
                           (id_ctrl.rxs2 && id_raddr2 =/= 0.U, id_raddr2),
                           (id_ctrl.wxd  && id_waddr  =/= 0.U, id_waddr))
  val fp_hazard_targets = Seq((io.fpu.dec.ren1, id_raddr1),
                              (io.fpu.dec.ren2, id_raddr2),
                              (io.fpu.dec.ren3, id_raddr3),
                              (io.fpu.dec.wen, id_waddr))

  val sboard = new Scoreboard(32, true)
  sboard.clear(ll_wen, ll_waddr)
  def id_sboard_clear_bypass(r: UInt) = {
    // ll_waddr arrives late when D$ has ECC, so reshuffle the hazard check
    if (!tileParams.dcache.get.dataECC.isDefined) ll_wen && ll_waddr === r
    else div.io.resp.fire && div.io.resp.bits.tag === r || dmem_resp_replay && dmem_resp_xpu && dmem_resp_waddr === r
  }
  val id_sboard_hazard = checkHazards(hazard_targets, rd => sboard.read(rd) && !id_sboard_clear_bypass(rd))
  sboard.set(wb_set_sboard && wb_wen, wb_waddr)

  // stall for RAW/WAW hazards on CSRs, loads, AMOs, and mul/div in execute stage.
  val ex_cannot_bypass = ex_ctrl.csr =/= CSR.N || ex_ctrl.jalr || ex_ctrl.mem || ex_ctrl.mul || ex_ctrl.div || ex_ctrl.fp || ex_ctrl.rocc || ex_scie_pipelined
  val data_hazard_ex = ex_ctrl.wxd && checkHazards(hazard_targets, _ === ex_waddr)
  val fp_data_hazard_ex = id_ctrl.fp && ex_ctrl.wfd && checkHazards(fp_hazard_targets, _ === ex_waddr)
  val id_ex_hazard = ex_reg_valid && (data_hazard_ex && ex_cannot_bypass || fp_data_hazard_ex)

  // stall for RAW/WAW hazards on CSRs, LB/LH, and mul/div in memory stage.
  val mem_mem_cmd_bh =
    if (fastLoadWord) (!fastLoadByte).B && mem_reg_slow_bypass
    else true.B
  val mem_cannot_bypass = mem_ctrl.csr =/= CSR.N || mem_ctrl.mem && mem_mem_cmd_bh || mem_ctrl.mul || mem_ctrl.div || mem_ctrl.fp || mem_ctrl.rocc
  val data_hazard_mem = mem_ctrl.wxd && checkHazards(hazard_targets, _ === mem_waddr)
  val fp_data_hazard_mem = id_ctrl.fp && mem_ctrl.wfd && checkHazards(fp_hazard_targets, _ === mem_waddr)
  val id_mem_hazard = mem_reg_valid && (data_hazard_mem && mem_cannot_bypass || fp_data_hazard_mem)
  id_load_use := mem_reg_valid && data_hazard_mem && mem_ctrl.mem

  // stall for RAW/WAW hazards on load/AMO misses and mul/div in writeback.
  val data_hazard_wb = wb_ctrl.wxd && checkHazards(hazard_targets, _ === wb_waddr)
  val fp_data_hazard_wb = id_ctrl.fp && wb_ctrl.wfd && checkHazards(fp_hazard_targets, _ === wb_waddr)
  val id_wb_hazard = wb_reg_valid && (data_hazard_wb && wb_set_sboard || fp_data_hazard_wb)

  val id_stall_fpu = if (usingFPU) {
    val fp_sboard = new Scoreboard(32)
    fp_sboard.set((wb_dcache_miss && wb_ctrl.wfd || io.fpu.sboard_set) && wb_valid, wb_waddr)
    fp_sboard.clear(dmem_resp_replay && dmem_resp_fpu, dmem_resp_waddr)
    fp_sboard.clear(io.fpu.sboard_clr, io.fpu.sboard_clra)

    /*runahead code begin*/
    //fp_sboard.clear(runahead_posedge.asBool && checkHazards(fp_hazard_targets, fp_sboard.read _), mshr_l2miss_tag)
    /*runahead code end*/

    checkHazards(fp_hazard_targets, fp_sboard.read _) 
  } else false.B

  /*runahead code beign*/
  dontTouch(id_stall_fpu)

  //define runahead FSM and the trigger of l2miss
  val inv_events = RegInit(0.U(2.W))
  val take_pc_mem_wb_reg = RegNext(take_pc_mem_wb)
  val runahead_enter = /*(id_stall_fpu || id_sboard_hazard)*/id_sboard_hazard && !io.dmem.l2hit && 
                    (id_raddr1 === mshr_l2miss_tag(6, 2) || 
                    id_raddr2 === mshr_l2miss_tag(6, 2) || 
                    id_waddr === mshr_l2miss_tag(6, 2)) &&
                    (mshr_l2miss_tag =/= 0.U) &&
                    io.dmem.mshr_flag

  val s1_reg_reg_io_dmem_req_bits_addr =RegNext(dmem_req_addr)
  val s2_reg_io_dmem_req_bits_addr = RegNext(s1_reg_reg_io_dmem_req_bits_addr)
  rcu.io.l2miss := runahead_posedge //从检测到发送该l2miss延迟了两个周期（可能在检测阶段有replay）


  s0_db_flag := runahead_state === s_pass && io.dmem.mshr_state(0) === 0.U &&
                io.dmem.resp.valid && io.dmem.resp.bits.tag(5, 1) === l2miss_tag(6, 2) &&
                io.dmem.resp.bits.addr === l2miss_addr
  when(runahead_state === s_pass && io.dmem.mshr_state(0) === 0.U/* && io.dmem.mshr_state(1) === 0.U*/) {
    db_flag := io.dmem.resp.valid && io.dmem.resp.bits.tag(5, 1) === l2miss_tag(6, 2) && io.dmem.resp.bits.addr === l2miss_addr
  } .otherwise {
    db_flag := false.B
  } 
  when(io.dmem.mshr_state(0) =/= 5.U) {
    inv_events := 0.U
  } .elsewhen(runahead_state === s_exit && io.dmem.mshr_state(0) === 5.U) {
    inv_events := 0.U
  } .elsewhen(runahead_state === s_runahead_wait_resp && s2_reg_io_dmem_req_bits_addr === io.dmem.resp.bits.addr && io.dmem.mshr_state(0) === 5.U && !io.dmem.resp.valid ) {
    inv_events := inv_events + 1.U
  }
//mshr 不写回
when(runahead_state === s_runahead) {
 when(l2cache_miss && s2_dcache_miss(1)){
  runahead_l2miss_events(1) := true.B
 }
 when(l2cache_miss && s2_dcache_miss(2)) {
  runahead_l2miss_events(2) := true.B
 }
 when(l2cache_miss && s2_dcache_miss(3)) {
  runahead_l2miss_events(3) := true.B
 }
}.elsewhen(runahead_state === s_runahead && io.dmem.resp.valid) {
  when(s1_mshr_deq_ptr_value(1) === runahead_enq_ptr_value(1) && io.dmem.resp.bits.addr>>6.U === runahead_addr(1)>>6.U && runahead_l2miss_events(1) === true.B ) {
    runahead_l2miss_events(1) := false.B
  }
  when(s1_mshr_deq_ptr_value(2) === runahead_enq_ptr_value(2) && io.dmem.resp.bits.addr>>6.U === runahead_addr(2)>>6.U && runahead_l2miss_events(2) === true.B ) {
    runahead_l2miss_events(2) := false.B
  }
  when(s1_mshr_deq_ptr_value(3) === runahead_enq_ptr_value(3) && io.dmem.resp.bits.addr>>6.U === runahead_addr(3)>>6.U && runahead_l2miss_events(3) === true.B ) {
    runahead_l2miss_events(3) := false.B
  }
}
.elsewhen(s2_db_flag && runahead_state === s_pass) {
  when(io.dmem.mshr_state(0) =/= 0.U) {
    runahead_l2miss_events(0) := true.B
  }
  when(io.dmem.mshr_state(1) =/= 0.U) {
    runahead_l2miss_events(1) := true.B
  }
  when(io.dmem.mshr_state(2) =/= 0.U) {
    runahead_l2miss_events(2) := true.B
  }
  when(io.dmem.mshr_state(3) =/= 0.U) {
    runahead_l2miss_events(3) :=true.B
  }
}.elsewhen(runahead_state === s_pseudo_exit && io.dmem.resp.valid) {
  when(s1_mshr_deq_ptr_value(0) === runahead_enq_ptr_value(0) && io.dmem.resp.bits.addr>>6.U === runahead_addr(0)>>6.U && runahead_l2miss_events(0) === true.B) {
    runahead_l2miss_events(0) := false.B
  }
  when(s1_mshr_deq_ptr_value(1) === runahead_enq_ptr_value(1) && io.dmem.resp.bits.addr>>6.U === runahead_addr(1)>>6.U && runahead_l2miss_events(1) === true.B ) {
    runahead_l2miss_events(1) := false.B
  }
  when(s1_mshr_deq_ptr_value(2) === runahead_enq_ptr_value(2) && io.dmem.resp.bits.addr>>6.U === runahead_addr(2)>>6.U && runahead_l2miss_events(2) === true.B ) {
    runahead_l2miss_events(2) := false.B
  }
  when(s1_mshr_deq_ptr_value(3) === runahead_enq_ptr_value(3) && io.dmem.resp.bits.addr>>6.U === runahead_addr(3)>>6.U && runahead_l2miss_events(3) === true.B ) {
    runahead_l2miss_events(3) := false.B
  }
}
//l2dcache miss signal
l2cache_miss := io.dmem.l2acquire && !RegNext(io.dmem.l2acquire)
for(i <- s1_mshr_state.indices) {
  dcache_miss(i) := (io.dmem.mshr_state(i) === 5.U) && (s1_mshr_state(i) === 4.U)
}
for(i <- s1_mshr_state.indices) {
  mshr_l2dcache_miss(i) := l2cache_miss && s2_dcache_miss(i)
}

dontTouch(l2cache_miss)
dontTouch(mshr_l2dcache_miss)
dontTouch(dcache_miss)
dontTouch(runahead_tag)
dontTouch(runahead_deq_ptr_value)
when(runahead_posedge) {
  l2miss_tag := mshr_l2miss_tag
  l2miss_addr := mshr_l2miss_addr
}.elsewhen(runahead_state === s_runahead) {
  when(l2cache_miss && s2_dcache_miss(1)) {
    runahead_tag(1) := io.dmem.mshr_tag(1)
    runahead_addr(1) := io.dmem.mshr_addr(1)
    runahead_enq_ptr_value(1) := io.dmem.mshr_enq_ptr_value(1)
    runahead_deq_ptr_value(1) := io.dmem.mshr_deq_ptr_value(1)
  }
  when(l2cache_miss && s2_dcache_miss(2)) {
    runahead_tag(2) := io.dmem.mshr_tag(2)
    runahead_addr(2) := io.dmem.mshr_addr(2)
    runahead_enq_ptr_value(2) := io.dmem.mshr_enq_ptr_value(2)
    runahead_deq_ptr_value(2) := io.dmem.mshr_deq_ptr_value(2)
  }
  when(l2cache_miss && s2_dcache_miss(3)) {
    runahead_tag(3) := io.dmem.mshr_tag(3)
    runahead_addr(3) := io.dmem.mshr_addr(3)
    runahead_enq_ptr_value(3) := io.dmem.mshr_enq_ptr_value(3)
    runahead_deq_ptr_value(3) := io.dmem.mshr_deq_ptr_value(3)
  }
}.elsewhen(s2_db_flag && runahead_state === s_pass) {
  when(io.dmem.mshr_state(0) =/= 0.U) {
    runahead_tag(0) := io.dmem.mshr_tag(0)
    runahead_addr(0) := io.dmem.mshr_addr(0)
    runahead_enq_ptr_value(0) := io.dmem.mshr_enq_ptr_value(0)
    runahead_deq_ptr_value(0) := io.dmem.mshr_deq_ptr_value(0)
  }
  when(io.dmem.mshr_state(1) =/= 0.U) {
    runahead_tag(1) := io.dmem.mshr_tag(1)
    runahead_addr(1) := io.dmem.mshr_addr(1)
    runahead_enq_ptr_value(1) := io.dmem.mshr_enq_ptr_value(1)
    runahead_deq_ptr_value(1) := io.dmem.mshr_deq_ptr_value(1)
  }
  when(io.dmem.mshr_state(2) =/= 0.U) {
    runahead_tag(2) := io.dmem.mshr_tag(2)
    runahead_addr(2) := io.dmem.mshr_addr(2)
    runahead_enq_ptr_value(2) := io.dmem.mshr_enq_ptr_value(2)
    runahead_deq_ptr_value(2) := io.dmem.mshr_deq_ptr_value(2)
  }
  when(io.dmem.mshr_state(3) =/= 0.U) {
    runahead_tag(3) := io.dmem.mshr_tag(3)
    runahead_addr(3) := io.dmem.mshr_addr(3)
    runahead_enq_ptr_value(3) := io.dmem.mshr_enq_ptr_value(3)
    runahead_deq_ptr_value(3) := io.dmem.mshr_deq_ptr_value(3)
  }
} .elsewhen(runahead_state === s_exit) {
  runahead_tag := VecInit(Seq.fill(4)(0.U(7.W)))
  runahead_addr := VecInit(Seq.fill(4)(0.U(40.W)))
  runahead_enq_ptr_value := VecInit(Seq.fill(4)(0.U(4.W)))
  runahead_deq_ptr_value := VecInit(Seq.fill(4)(0.U(4.W)))
  l2miss_tag := 0.U
  l2miss_addr := 0.U
}

  when(runahead_state === s_runahead_wait_req && io.dmem.req.valid && io.dmem.mshr_state(0) === 5.U) {
    runahead_state := s_runahead_wait_resp
  }
  when(runahead_state === s_runahead_wait_resp && s2_reg_io_dmem_req_bits_addr === io.dmem.resp.bits.addr && io.dmem.mshr_state(0) === 5.U) {
    runahead_state := s_runahead_wait_req
  }
  when(runahead_state === s_runahead_wait_resp && io.dmem.mshr_state(0) =/=5.U) {
    runahead_state := s_runahead_wait_req
  }
//进入runahead的条件
  when(runahead_state === s_runahead_wait_req && runahead_enter && !take_pc_mem_wb_reg && !io.dmem.req.valid && io.dmem.mshr_cmd(0) === 0.U && !io.dmem.alloc_arb_out_ready
      && io.dmem.mshr_state(1) === 0.U && io.dmem.mshr_state(2) === 0.U && io.dmem.mshr_state(3) === 0.U
  ) {
    when(inv_events === 0.U) {
      runahead_state := s_runahead
    } .otherwise {
      runahead_state := s_inv
    }
  }
  when(runahead_state === s_runahead && io.dmem.mshr_state(0) === 8.U) {
    runahead_state := s_pass
  }

  // when( && runahead_state === s_runahead) {
  //     runahead_state := s_inv
  //   }
  when(s2_db_flag && runahead_state === s_pass) {
    when(io.dmem.mshr_state(0) === 0.U && io.dmem.mshr_state(1) === 0.U && io.dmem.mshr_state(2) === 0.U && io.dmem.mshr_state(3) === 0.U) {
      runahead_state := s_exit
    } .otherwise {
      runahead_state := s_pseudo_exit
    }
  }
  when(runahead_state === s_pseudo_exit && runahead_l2miss_events.orR === false.B) {
    runahead_state := s_exit
  }

  when(runahead_state === s_inv && io.dmem.resp.valid) {
    runahead_state := s_exit
  }
  when(runahead_state === s_exit && io.dmem.mshr_state(0) === 5.U) {
    runahead_state := s_runahead_wait_req
  }
  dontTouch(inv_events)
  dontTouch(runahead_enter)
  dontTouch(runahead_posedge)
  
  dontTouch(id_ex_hazard)
  dontTouch(id_mem_hazard)
  dontTouch(id_wb_hazard)
  dontTouch(wb_waddr)

   runahead_flag :=   rcu.io.runahead_flag
   io.dmem.runahead_flag := runahead_flag
  /*runahead code end*/

  /*runahead invfile beign*/
  val ex_raddr1 = ex_reg_inst(19, 15)
  val ex_raddr2 = ex_reg_inst(24, 20)
  dontTouch(ex_raddr1)
  dontTouch(ex_raddr2)
  dontTouch(ex_waddr)
  val rh_hazard_targets = Seq((ex_ctrl.rxs1 && ex_raddr1 =/= 0.U, ex_raddr1),
                              (ex_ctrl.rxs2 && ex_raddr2 =/= 0.U, ex_raddr2))

  sboard.clear((runahead_flag === true.B) && s2_runahead_posedge, l2miss_tag(6,2))//pretend stall for dependency
  //exit when l2miss stall in runahead(maybe mshr_l2dcahe_miss拉高之前，多个tag写入了mshr)
  sboard.clear(runahead_flag && mshr_l2dcache_miss(1),runahead_tag(1)(6,2))
  sboard.clear(runahead_flag && mshr_l2dcache_miss(2),runahead_tag(2)(6,2))
  sboard.clear(runahead_flag && mshr_l2dcache_miss(3),runahead_tag(3)(6,2))
  val rf_invfile = new Scoreboard(32, true)
  rf_invfile.set((runahead_flag === true.B) && s1_runahead_posedge,id_waddr) //导致进入l2missstall的inst 对应的目标寄存器置无效(可能不是寄存器依赖，是set match依赖)
  rf_invfile.set((runahead_flag === true.B) && s1_runahead_posedge, l2miss_tag(6,2))
  //set the inv_rf of l2miss_stall
  rf_invfile.set((runahead_flag === true.B) && mshr_l2dcache_miss(1), runahead_tag(1)(6,2))
  rf_invfile.set((runahead_flag === true.B) && mshr_l2dcache_miss(2), runahead_tag(2)(6,2))
  rf_invfile.set((runahead_flag === true.B) && mshr_l2dcache_miss(3), runahead_tag(3)(6,2))

  val ex_rfinvfile_propogation = checkHazards(rh_hazard_targets, rd => rf_invfile.read(rd))

  rf_invfile.set((runahead_flag === true.B) && ex_rfinvfile_propogation || rfinvfile_block_miss.orR, ex_waddr) //although invalid, it will still writeback to rf
  val rf_invfile_clear = Reg(Bool())
  rf_invfile_clear := !ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay
  rf_invfile.clear((runahead_flag === true.B) && !ex_rfinvfile_propogation && !s1_runahead_posedge && rf_invfile_clear && !rfinvfile_block_miss.orR, ex_waddr) // to do: add a ctrl_sig, judge if a load addr is valid
  val invfile = WireInit(VecInit(Seq.fill(31)(0.U(1.W))))
  for(i <- 0 until 31) {
    invfile(i) := rf_invfile.read(i.U).asUInt()
  }
  dontTouch(invfile)
  dontTouch(id_raddr1)
  dontTouch(id_raddr2)
  dontTouch(id_waddr)
  dontTouch(ex_rfinvfile_propogation)
  dontTouch(io.dmem.l2acquire)
  /*runahead invfile end*/

  /*runahead buffer beign*/
  //define array and signal
  val rhbuffer_addr = RegInit(VecInit(Seq.fill(10)(0.U(40.W))))
  val wr_ptr = RegInit(0.U(5.W))
  val addr_stobit = new Scoreboard(10, true)
  val addr_invfile = new Scoreboard(10, true)

  //hit? and find hit_ptr
  ex_rh_hit := MuxCase(0.U,Seq(
    ((rhbuffer_addr(0) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(0) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(1) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(1) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(2) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(2) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(3) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(3) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(4) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(4) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(5) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(5) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(6) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(6) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(7) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(7) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(8) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(8) =/= 0.U)) -> true.B,
    ((rhbuffer_addr(9) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(9) =/= 0.U)) -> true.B,
  ))

  ex_hit_ptr := MuxCase(0.U,Seq(
    ((rhbuffer_addr(0) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(0) =/= 0.U)) -> 0.U,
    ((rhbuffer_addr(1) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(1) =/= 0.U)) -> 1.U,
    ((rhbuffer_addr(2) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(2) =/= 0.U)) -> 2.U,
    ((rhbuffer_addr(3) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(3) =/= 0.U)) -> 3.U,
    ((rhbuffer_addr(4) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(4) =/= 0.U)) -> 4.U,
    ((rhbuffer_addr(5) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(5) =/= 0.U)) -> 5.U,
    ((rhbuffer_addr(6) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(6) =/= 0.U)) -> 6.U,
    ((rhbuffer_addr(7) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(7) =/= 0.U)) -> 7.U,
    ((rhbuffer_addr(8) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(8) =/= 0.U)) -> 8.U,
    ((rhbuffer_addr(9) === dmem_req_addr) && ((ex_rh_load === true.B) || (ex_rh_store === true.B)) && (rhbuffer_addr(9) =/= 0.U)) -> 9.U,
  ))

  // val addr_matches = rhbuffer_addr.zipWithIndex.map { case (addr, idx) =>
  // (addr === dmem_req_addr && addr_invfile.read === 1.U).asUInt << idx
  // }.reduce(_ | _)
  // ex_rh_hit := addr_matches.orR
  // val hit_ptr = (0 until 9).find { idx =>
  // (addr_matches >> idx).asUInt(0) === 1.U
  // }

  //store
  when((ex_rh_store === true.B) && (runahead_flag === true.B) && (ex_rh_hit === false.B) ){//store the data
    val size = Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_reg_mem_size)
    rhbuffer_addr(wr_ptr) := dmem_req_addr
    rhbuffer_data(wr_ptr) := new StoreGen(size, 0.U, ex_rs(1), coreDataBytes).data
    wr_ptr := Mux(wr_ptr === 9.U, 0.U, wr_ptr + 1.U)
  }.elsewhen((ex_rh_store === true.B) && (runahead_flag === true.B) && (ex_rh_hit === true.B)){//replace the data
    val size = Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_reg_mem_size)
    rhbuffer_data(ex_hit_ptr) := new StoreGen(size, 0.U, ex_rs(1), coreDataBytes).data
  }
  //invalid addr set and clear
  // addr_invfile.set(runahead_flag && runahead_posedge, l2miss_tag(6, 2))
  // addr_invfile.clear(runahead_flag && runahead_posedge, l2miss_tag(6, 2))

  //load -> send this data to register file

  //clear data and addr when exit runahead
  when(s2_db_flag){
    for (i <- 0 until 9) {
      rhbuffer_addr(i) := 0.U
      rhbuffer_data(i) := 0.U
      addr_invfile.clear(s2_db_flag, i.U)
      wr_ptr := 0.U
    }
    for (i <- 1 until 31) {
      rf_invfile.clear(s2_db_flag, i.U)
    }
  }

  /*runahead buffer end*/

    /*runahead code begin*/
  for (i <- 0 until 31) {
    rcu.io.sb_in(i) := sboard.read(i.U).asUInt()
  }
  when(s2_db_flag){
    for (j <- 0 until 31) {
      sboard.clear(rcu.io.sb_out(j) === false.B || 
                  (rcu.io.sb_out(j) === true.B && (j.U === l2miss_tag(6, 2))) ,j.U) //reset the sb_value that is resp_waddr
      sboard.set(rcu.io.sb_out(j) === true.B && (j.U =/= l2miss_tag(6, 2)) ,j.U) 
    }
  }
  
  block_addr(0) := l2miss_addr(39,6)
  block_addr(1) := runahead_addr(1)(39,6)
  block_addr(2) := runahead_addr(2)(39,6)
  block_addr(3) := runahead_addr(3)(39,6)
  ex_dmem_req_inv := ex_rh_hit || ex_rh_load && ex_rfinvfile_propogation || rfinvfile_block_miss.orR
  val s1_dmem_l2hit = RegNext(io.dmem.l2hit, init = true.B)
  val l2hit_nesedge = s1_dmem_l2hit && !io.dmem.l2hit
  val idx_match_l2miss = l2hit_nesedge && io.dmem.mshr_state(0) === 5.U && io.dmem.idx_match
  dontTouch(idx_match_l2miss)
  /*runahead code end*/
  val dcache_blocked = {
    // speculate that a blocked D$ will unblock the cycle after a Grant
    val blocked = Reg(Bool())
    blocked := !io.dmem.req.ready && io.dmem.clock_enabled && !io.dmem.perf.grant && (blocked || io.dmem.req.valid || io.dmem.s2_nack)
    blocked && !io.dmem.perf.grant
  }
  val rocc_blocked = Reg(Bool())
  rocc_blocked := !wb_xcpt && !io.rocc.cmd.ready && (io.rocc.cmd.valid || rocc_blocked)

  val ctrl_stalld =
    id_ex_hazard || id_mem_hazard || id_wb_hazard || id_sboard_hazard ||
    csr.io.singleStep && (ex_reg_valid || mem_reg_valid || wb_reg_valid) ||
    id_csr_en && csr.io.decode(0).fp_csr && !io.fpu.fcsr_rdy ||
    id_ctrl.fp && id_stall_fpu ||
    id_ctrl.mem && dcache_blocked || // reduce activity during D$ misses
    id_ctrl.rocc && rocc_blocked || // reduce activity while RoCC is busy
    id_ctrl.div && (!(div.io.req.ready || (div.io.resp.valid && !wb_wxd)) || div.io.req.valid) || // reduce odds of replay
    !clock_en ||
    id_do_fence ||
    csr.io.csr_stall ||
    id_reg_pause ||
    io.traceStall ||
    /*runahead code begin*/
    s0_db_flag || db_flag || s1_db_flag || s2_db_flag
    /*runahead code end*/
  ctrl_killd := (!ibuf.io.inst(0).valid || ibuf.io.inst(0).bits.replay || take_pc_mem_wb || ctrl_stalld || csr.io.interrupt) //|| 
  /*runahead code begin*/
  //exit_miss_back
  /*runahead code end*/

  io.imem.req.valid := take_pc
  io.imem.req.bits.speculative := !take_pc_wb
  io.imem.req.bits.pc :=
    Mux(wb_xcpt || csr.io.eret, csr.io.evec, // exception or [m|s]ret
    /*runahead code begin*/
//    Mux(s2_runahead_posedge, wb_reg_pc,
    Mux(db_flag ,rcu.io.opc,
    /*runahead code end*/   
    Mux(replay_wb,              wb_reg_pc,   // replay


                                mem_npc)))//)    // flush or branch misprediction
  io.imem.flush_icache := wb_reg_valid && wb_ctrl.fence_i && !io.dmem.s2_nack
  io.imem.might_request := {
    imem_might_request_reg := ex_pc_valid || mem_pc_valid || io.ptw.customCSRs.disableICacheClockGate
    imem_might_request_reg
  }
  io.imem.progress := RegNext(wb_reg_valid && !replay_wb_common)
  io.imem.sfence.valid := wb_reg_valid && wb_reg_sfence
  io.imem.sfence.bits.rs1 := wb_reg_mem_size(0)
  io.imem.sfence.bits.rs2 := wb_reg_mem_size(1)
  io.imem.sfence.bits.addr := wb_reg_wdata
  io.imem.sfence.bits.asid := wb_reg_rs2
  io.imem.sfence.bits.hv := wb_reg_hfence_v
  io.imem.sfence.bits.hg := wb_reg_hfence_g
  io.ptw.sfence := io.imem.sfence

  ibuf.io.inst(0).ready := !ctrl_stalld

  io.imem.btb_update.valid := mem_reg_valid && !take_pc_wb && mem_wrong_npc && (!mem_cfi || mem_cfi_taken)
  io.imem.btb_update.bits.isValid := mem_cfi
  io.imem.btb_update.bits.cfiType :=
    Mux((mem_ctrl.jal || mem_ctrl.jalr) && mem_waddr(0), CFIType.call,
    Mux(mem_ctrl.jalr && (mem_reg_inst(19,15) & regAddrMask.U) === BitPat("b00?01"), CFIType.ret,
    Mux(mem_ctrl.jal || mem_ctrl.jalr, CFIType.jump,
    CFIType.branch)))
  io.imem.btb_update.bits.target := io.imem.req.bits.pc
  io.imem.btb_update.bits.br_pc := (if (usingCompressed) mem_reg_pc + Mux(mem_reg_rvc, 0.U, 2.U) else mem_reg_pc)
  io.imem.btb_update.bits.pc := ~(~io.imem.btb_update.bits.br_pc | (coreInstBytes*fetchWidth-1).U)
  io.imem.btb_update.bits.prediction := mem_reg_btb_resp

  io.imem.bht_update.valid := mem_reg_valid && !take_pc_wb
  io.imem.bht_update.bits.pc := io.imem.btb_update.bits.pc
  io.imem.bht_update.bits.taken := mem_br_taken
  io.imem.bht_update.bits.mispredict := mem_wrong_npc
  io.imem.bht_update.bits.branch := mem_ctrl.branch
  io.imem.bht_update.bits.prediction := mem_reg_btb_resp.bht

  io.fpu.valid := !ctrl_killd && id_ctrl.fp
  io.fpu.killx := ctrl_killx
  io.fpu.killm := killm_common
  io.fpu.inst := id_inst(0)
  io.fpu.fromint_data := ex_rs(0)
  io.fpu.dmem_resp_val := dmem_resp_valid && dmem_resp_fpu
  io.fpu.dmem_resp_data := (if (minFLen == 32) io.dmem.resp.bits.data_word_bypass else io.dmem.resp.bits.data)
  io.fpu.dmem_resp_type := io.dmem.resp.bits.size
  io.fpu.dmem_resp_tag := dmem_resp_waddr
  io.fpu.keep_clock_enabled := io.ptw.customCSRs.disableCoreClockGate

  io.dmem.req.valid     := ex_reg_valid && ex_ctrl.mem &&
  /*runahead code begin*/
  !ex_rh_store &&                             // rh_store will never send request to dcache
  !ex_dmem_req_inv
  // !(ex_rh_hit && ex_rh_load) &&               //if rh_load hits in runahead buffer, the request will not send to dcache
  // !(ex_rfinvfile_propogation && ex_rh_load) //if load reqaddr is invalid, will not send request
  dmem_req_addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out)
  //same_tag_miss := runahead_flag && l2miss_tag(6, 2) === ex_waddr
  /*runahead code end*/
  val ex_dcache_tag = Cat(ex_waddr, ex_ctrl.fp)
  require(coreParams.dcacheReqTagBits >= ex_dcache_tag.getWidth)
  //io.dmem.req.bits.tag  := ex_dcache_tag
  /*runahead code begin*/
  io.dmem.req.bits.tag  := Mux(rfinvfile_block_miss.orR, 0.U, ex_dcache_tag)
  /*runahead code end*/
  io.dmem.req.bits.cmd  := ex_ctrl.mem_cmd
  io.dmem.req.bits.size := ex_reg_mem_size
  io.dmem.req.bits.signed := !Mux(ex_reg_hls, ex_reg_inst(20), ex_reg_inst(14))
  io.dmem.req.bits.phys := false.B
  //io.dmem.req.bits.addr := encodeVirtualAddress(ex_rs(0), alu.io.adder_out)
  /*runahead code begin*/
  io.dmem.req.bits.addr := Mux(rfinvfile_block_miss.orR, 0.U, encodeVirtualAddress(ex_rs(0), alu.io.adder_out))
  /*runahead code end*/
  io.dmem.req.bits.idx.foreach(_ := io.dmem.req.bits.addr)
  io.dmem.req.bits.dprv := Mux(ex_reg_hls, csr.io.hstatus.spvp, csr.io.status.dprv)
  io.dmem.req.bits.dv := ex_reg_hls || csr.io.status.dv
  io.dmem.s1_data.data := (if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2))
  io.dmem.s1_kill := killm_common || mem_ldst_xcpt || fpu_kill_mem
  io.dmem.s2_kill := false.B
  // don't let D$ go to sleep if we're probably going to use it soon
  io.dmem.keep_clock_enabled := ibuf.io.inst(0).valid && id_ctrl.mem && !csr.io.csr_stall

  io.rocc.cmd.valid := wb_reg_valid && wb_ctrl.rocc && !replay_wb_common
  io.rocc.exception := wb_xcpt && csr.io.status.xs.orR
  io.rocc.cmd.bits.status := csr.io.status
  io.rocc.cmd.bits.inst := wb_reg_inst.asTypeOf(new RoCCInstruction())
  io.rocc.cmd.bits.rs1 := wb_reg_wdata
  io.rocc.cmd.bits.rs2 := wb_reg_rs2

  // gate the clock
  val unpause = csr.io.time(rocketParams.lgPauseCycles-1, 0) === 0.U || csr.io.inhibit_cycle || io.dmem.perf.release || take_pc
  when (unpause) { id_reg_pause := false.B }
  io.cease := csr.io.status.cease && !clock_en_reg
  io.wfi := csr.io.status.wfi
  if (rocketParams.clockGate) {
    long_latency_stall := csr.io.csr_stall || io.dmem.perf.blocked || id_reg_pause && !unpause
    clock_en := clock_en_reg || ex_pc_valid || (!long_latency_stall && io.imem.resp.valid)
    clock_en_reg :=
      ex_pc_valid || mem_pc_valid || wb_pc_valid || // instruction in flight
      io.ptw.customCSRs.disableCoreClockGate || // chicken bit
      !div.io.req.ready || // mul/div in flight
      usingFPU.B && !io.fpu.fcsr_rdy || // long-latency FPU in flight
      io.dmem.replay_next || // long-latency load replaying
      (!long_latency_stall && (ibuf.io.inst(0).valid || io.imem.resp.valid)) // instruction pending

    assert(!(ex_pc_valid || mem_pc_valid || wb_pc_valid) || clock_en)
  }

  // evaluate performance counters
  val icache_blocked = !(io.imem.resp.valid || RegNext(io.imem.resp.valid))
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }

  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))

  coreMonitorBundle.clock := clock
  coreMonitorBundle.reset := reset
  coreMonitorBundle.hartid := io.hartid
  coreMonitorBundle.timer := csr.io.time(31,0)
  coreMonitorBundle.valid := csr.io.trace(0).valid && !csr.io.trace(0).exception
  coreMonitorBundle.pc := csr.io.trace(0).iaddr(vaddrBitsExtended-1, 0).sextTo(xLen)
  coreMonitorBundle.wrenx := wb_wen && !wb_set_sboard
  coreMonitorBundle.wrenf := false.B
  coreMonitorBundle.wrdst := wb_waddr
  coreMonitorBundle.wrdata := rf_wdata
  coreMonitorBundle.rd0src := wb_reg_inst(19,15)
  coreMonitorBundle.rd0val := RegNext(RegNext(ex_rs(0)))
  coreMonitorBundle.rd1src := wb_reg_inst(24,20)
  coreMonitorBundle.rd1val := RegNext(RegNext(ex_rs(1)))
  coreMonitorBundle.inst := csr.io.trace(0).insn
  coreMonitorBundle.excpt := csr.io.trace(0).exception
  coreMonitorBundle.priv_mode := csr.io.trace(0).priv

  if (enableCommitLog) {
    val t = csr.io.trace(0)
    val rd = wb_waddr
    val wfd = wb_ctrl.wfd
    val wxd = wb_ctrl.wxd
    val has_data = wb_wen && !wb_set_sboard

    when (t.valid && !t.exception) {
      when (wfd) {
        printf ("%d 0x%x (0x%x) f%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd+32.U)
      }
      .elsewhen (wxd && rd =/= 0.U && has_data) {
        printf ("%d 0x%x (0x%x) x%d 0x%x\n", t.priv, t.iaddr, t.insn, rd, rf_wdata)
      }
      .elsewhen (wxd && rd =/= 0.U && !has_data) {
        printf ("%d 0x%x (0x%x) x%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd)
      }
      .otherwise {
        printf ("%d 0x%x (0x%x)\n", t.priv, t.iaddr, t.insn)
      }
    }

    when (ll_wen && rf_waddr =/= 0.U) {
      printf ("x%d p%d 0x%x\n", rf_waddr, rf_waddr, rf_wdata)
    }
  }
  else {
    when (csr.io.trace(0).valid) {
      printf("C%d: %d [%d] pc=[%x] W[r%d=%x][%d] R[r%d=%x] R[r%d=%x] inst=[%x] DASM(%x)\n",
         io.hartid, coreMonitorBundle.timer, coreMonitorBundle.valid,
         coreMonitorBundle.pc,
         Mux(wb_ctrl.wxd || wb_ctrl.wfd, coreMonitorBundle.wrdst, 0.U),
         Mux(coreMonitorBundle.wrenx, coreMonitorBundle.wrdata, 0.U),
         coreMonitorBundle.wrenx,
         Mux(wb_ctrl.rxs1 || wb_ctrl.rfs1, coreMonitorBundle.rd0src, 0.U),
         Mux(wb_ctrl.rxs1 || wb_ctrl.rfs1, coreMonitorBundle.rd0val, 0.U),
         Mux(wb_ctrl.rxs2 || wb_ctrl.rfs2, coreMonitorBundle.rd1src, 0.U),
         Mux(wb_ctrl.rxs2 || wb_ctrl.rfs2, coreMonitorBundle.rd1val, 0.U),
         coreMonitorBundle.inst, coreMonitorBundle.inst)
    }
  }

  // CoreMonitorBundle for late latency writes
  val xrfWriteBundle = Wire(new CoreMonitorBundle(xLen, fLen))

  xrfWriteBundle.clock := clock
  xrfWriteBundle.reset := reset
  xrfWriteBundle.hartid := io.hartid
  xrfWriteBundle.timer := csr.io.time(31,0)
  xrfWriteBundle.valid := false.B
  xrfWriteBundle.pc := 0.U
  xrfWriteBundle.wrdst := rf_waddr
  xrfWriteBundle.wrenx := rf_wen && !(csr.io.trace(0).valid && wb_wen && (wb_waddr === rf_waddr))
  xrfWriteBundle.wrenf := false.B
  xrfWriteBundle.wrdata := rf_wdata
  xrfWriteBundle.rd0src := 0.U
  xrfWriteBundle.rd0val := 0.U
  xrfWriteBundle.rd1src := 0.U
  xrfWriteBundle.rd1val := 0.U
  xrfWriteBundle.inst := 0.U
  xrfWriteBundle.excpt := false.B
  xrfWriteBundle.priv_mode := csr.io.trace(0).priv

  PlusArg.timeout(
    name = "max_core_cycles",
    docstring = "Kill the emulation after INT rdtime cycles. Off if 0."
  )(csr.io.time)

  } // leaving gated-clock domain
  val rocketImpl = withClock (gated_clock) { new RocketImpl }

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  def coverExceptions(exceptionValid: Bool, cause: UInt, labelPrefix: String, coverCausesLabels: Seq[(Int, String)]): Unit = {
    for ((coverCause, label) <- coverCausesLabels) {
      property.cover(exceptionValid && (cause === coverCause.U), s"${labelPrefix}_${label}")
    }
  }

  def checkHazards(targets: Seq[(Bool, UInt)], cond: UInt => Bool) =
    targets.map(h => h._1 && cond(h._2)).reduce(_||_)

  def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) ea else {
    // efficient means to compress 64-bit VA into vaddrBits+1 bits
    // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1))
    val b = vaddrBitsExtended-1
    val a = (a0 >> b).asSInt
    val msb = Mux(a === 0.S || a === -1.S, ea(b), !ea(b-1))
    Cat(msb, ea(b-1, 0))
  }

  class Scoreboard(n: Int, zero: Boolean = false)
  {
    def set(en: Bool, addr: UInt): Unit = update(en, _next | mask(en, addr))
    def clear(en: Bool, addr: UInt): Unit = update(en, _next & ~mask(en, addr))
    def read(addr: UInt): Bool = r(addr)
    def readBypassed(addr: UInt): Bool = _next(addr)

    private val _r = RegInit(0.U(n.W))
    private val r = if (zero) (_r >> 1 << 1) else _r
    private var _next = r
    private var ens = false.B
    private def mask(en: Bool, addr: UInt) = Mux(en, 1.U << addr, 0.U)
    private def update(en: Bool, update: UInt) = {
      _next = update
      ens = ens || en
      when (ens) { _r := _next }
    }
  }
}

class RegFile(n: Int, w: Int, zero: Boolean = false) {
  val rf = Mem(n, UInt(w.W))
  private def access(addr: UInt) = rf(~addr(log2Up(n)-1,0))
  private val reads = ArrayBuffer[(UInt,UInt)]()
  private var canRead = true
  def read(addr: UInt) = {
    require(canRead)
    reads += addr -> Wire(UInt())
    reads.last._2 := Mux(zero.B && addr === 0.U, 0.U, access(addr))
    reads.last._2
  }
  def write(addr: UInt, data: UInt) = {
    canRead = false
    when (addr =/= 0.U) {
      access(addr) := data
      for ((raddr, rdata) <- reads)
        when (addr === raddr) { rdata := data }
    }
  }
}

object ImmGen {
  def apply(sel: UInt, inst: UInt) = {
    val sign = Mux(sel === IMM_Z, 0.S, inst(31).asSInt)
    val b30_20 = Mux(sel === IMM_U, inst(30,20).asSInt, sign)
    val b19_12 = Mux(sel =/= IMM_U && sel =/= IMM_UJ, sign, inst(19,12).asSInt)
    val b11 = Mux(sel === IMM_U || sel === IMM_Z, 0.S,
              Mux(sel === IMM_UJ, inst(20).asSInt,
              Mux(sel === IMM_SB, inst(7).asSInt, sign)))
    val b10_5 = Mux(sel === IMM_U || sel === IMM_Z, 0.U, inst(30,25))
    val b4_1 = Mux(sel === IMM_U, 0.U,
               Mux(sel === IMM_S || sel === IMM_SB, inst(11,8),
               Mux(sel === IMM_Z, inst(19,16), inst(24,21))))
    val b0 = Mux(sel === IMM_S, inst(7),
             Mux(sel === IMM_I, inst(20),
             Mux(sel === IMM_Z, inst(15), 0.U)))

    Cat(sign, b30_20, b19_12, b11, b10_5, b4_1, b0).asSInt
  }
}
