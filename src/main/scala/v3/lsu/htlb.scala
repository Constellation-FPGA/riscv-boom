package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import freechips.rocketchip.rocket
import boom.v3.common._

/** A request made to an HTLB.
 *
 * Requires the "handle address" to operate on and the command (CMD) that will
 * be performed.
 */
class HTLBReq(implicit p: Parameters) extends BoomBundle()(p) {
  val haddr = UInt(xLen.W)
  val cmd = Bits(rocket.M_SZ.W)
  /** Should this request "passthrough" the HTLB and just return? */
  val passthrough = Bool()
}

/** Exception types that the HTLB can raise for a handle fault.
 *
 * NOTE: You should not rely on this being a [[Bundle]]. It may become an
 * [[Enum]] in future iterations.
 */
class HTLBExceptions extends Bundle {
  val ld = Bool()
  val st = Bool()
}

/** Response from HTLB for a handle that was translated to a virtual address.
 *
 * This indicates whether or not the translation was a miss in the HTLB, which
 * is communicated back to the LSU to send out a full virtual memory request to
 * L1d$ (and therefore memory) to translate the handle.
 *
 * This will return the virtual address of the handle if the translation was
 * not a miss.
 */
class HTLBResp(implicit p: Parameters) extends BoomBundle()(p) {
  /** The address that corresponds to the handle given in the request.
   * This may be a virtual address or a physical address, depending on the
   * value of [[HTLBResp.phys]].
   * If it is a virtual address, then it must go through page translation before
   * the actual-requested memory operation happens. */
  val addr = UInt(maxSVAddrBits.W)
  val miss = Bool()
  /** If the handle is not present or an load/store was attempted with the wrong
   * access permissions on the handle, a handle fault will be raised. The kind
   * of handle fault is given by HTLBExceptions. */
  val handle_fault = new HTLBExceptions
  /** States if [[HTLBResp.vaddr]] is already a physical address. */
  val phys = Bool()
  /** Entry is allowed to "try to upgrade" to a physical mapping. */
  val try_phys = Bool()
}

/** Describes the configuration of an [[HTLB]].
 *
 * @param nSets The number of sets available inside this HTLB.
 * @param nWays The number of ways available inside each set. Each way is identical.
 */
case class HTLBConfig(
  nSets: Int,
  nWays: Int,
)

/** Handle Translation Lookaside Buffer.
 *
 * This functions similarly to the translation lookaside buffer (TLB) used for
 * virtual memory. Instead of handling virtual address pages, this operates on
 * and works with handles.
 *
 * This is an abstract class that describes how all HTLBs interface with other
 * modules.
 */
abstract class HTLB(cfg: HTLBConfig)(implicit p: Parameters)
  extends BoomModule()(p) {
  val io = IO(new Bundle {
    val req = Flipped(Vec(memWidth, Decoupled(new HTLBReq)))
    val resp = Vec(memWidth, new HTLBResp)
    /** The size of the handle table in memory in bytes. */
    val htSize = Input(UInt(xLen.W))
  })

  /* Ensure that when we receive a valid request, the payload (handle address)
   * is ACTUALLY a handle. */
  for (w <- 0 until memWidth) {
    val req = io.req(w)
    assert(implies(req.fire, is_handle(req.bits.haddr)),
      "[htlb] HTLB translation requests must have addresses be handles")
  }
}

/** Handle Translation Lookaside Buffer that always produces handle faults.
 *
 * This is provided as an option because it is always valid for a handle to not
 * be translate-able by hardware and need to fall back to software to do the
 * right thing.
 * This is similar in spirit to how a hardware page table walker might give up
 * and require the operating system kernel to manually do a page table walk.
 */
class FaultingHTLB(cfg: HTLBConfig)(implicit p: Parameters) extends HTLB(cfg)(p) {
  for (w <- 0 until memWidth) {
    val req = io.req(w)

    /* Stub translation result.
     *
     * Every hardware translation is just immediately a handle fault. This lets
     * us exercise the fault logic without needing the hardware to spend the
     * time, connect to other modules, and actually do the work of
     * translation. This also means this module is always ready. */
    req.ready := true.B
    val present = false.B
    val r_perm  = false.B
    val w_perm  = false.B
    val addr    = 0.U(xLen.W)

    val cmd_read  = rocket.isRead(req.bits.cmd)
    val cmd_write = rocket.isWrite(req.bits.cmd)

    /* Despite this module always returning a handle fault, a handle fault is
     * NOT a miss! A miss means that the handle was valid, but that the HTLB
     * has not previously (up to replacement policies) seen that handle and
     * therefore does not have the handle in its HTLB. */
    io.resp(w).miss  := false.B
    io.resp(w).addr := addr

    /* TODO: Remove physical handle translation results! */
    io.resp(w).phys := false.B
    io.resp(w).try_phys := false.B

    /* NOTE: If a handle is marked as writable, then this block of hardware
     * will also IMPLICITLY treat it as readable.
     * This is generally the idea, because what good is a handle structure
     * that you can write to that you cannot read from? */
    io.resp(w).handle_fault.ld := req.valid && cmd_read  && (!present || !r_perm)
    io.resp(w).handle_fault.st := req.valid && cmd_write && (!present || !w_perm)
  }
}

/** Class meant for debugging the HTLB with Perfetto.
 *
 * Uses Midas (Golden Gate) synthesized printf's so that FireSim simulations
 * can be played into Perfetto for visualization and debugging.
 *
 */
class TimelineTracker()(implicit val p: Parameters) extends HasBoomCoreParameters {
  val cycle = if (boomParams.enableStateTracing) {
    val c = RegInit(0.U(64.W))
    c := c + 1.U
    c
  } else { 0.U }

  def mark(thread: String, event: String): Unit = {
    if (boomParams.enableStateTracing) {
      midas.targetutils.SynthesizePrintf(printf(s"TL(M,$thread,$event,%d)\n", cycle))
    }
  }

  def trackStateValue(thread: String, stateName: String, stateValue: UInt, sig: Bool): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle  = RegInit(0.U(64.W))
      val valueAtStart = RegInit(0.U(stateValue.getWidth.W))
      val prev        = RegNext(sig, false.B)
      when(sig && !prev) {
        startCycle   := cycle
        valueAtStart := stateValue
      }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$stateName %x,%d,%d,0)\n", valueAtStart, cycle, startCycle)
        )
      }
    }
  }

  def trackState(thread: String, event: String, sig: Bool, value: UInt = 0.U): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle = RegInit(0.U(64.W))
      val prev       = RegNext(sig, false.B)
      when(sig && !prev) { startCycle := cycle }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$event,%d,%d,%d)\n", cycle, startCycle, value)
        )
      }
    }
  }

  def trackCounter(thread: String, count: UInt): Unit = {
    if (boomParams.enableStateTracing) {
      val prev = RegNext(count, 0.U)
      when(count =/= prev) {
        midas.targetutils.SynthesizePrintf(printf(s"TL(C,$thread,%d,%d)\n", cycle, count))
      }
    }
  }
}
