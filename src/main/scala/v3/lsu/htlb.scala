package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.util.Split

import boom.v3.common._

/** A request made to an HTLB.
 *
 * Requires the "handle address" to operate on and the command (CMD) that will
 * be performed.
 */
class HTLBReq(implicit p: Parameters) extends BoomBundle()(p) {
  val haddr = new Handle()
  val cmd = Bits(rocket.M_SZ.W)
  /** Should this request "passthrough" the HTLB and just return?
   *
   * This is used for translation requests that go to the HTLB for addresses
   * ([[HTLBReq.haddr]]) that are virtual addresses and NOT actually handles.
   * This is set up this way because of how Yukon needs to interoperate with
   * BOOM's LSU and the TLB.
   *
   * Any passthrough request should be ignored by the HTLB and an [[HTLBResp]]
   * immediately returned where [[HTLBResp.miss]] is false.B and
   * [[HTLBResp.addr]] is the given address converted to a virtual address.
   * */
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
   * the actual-requested memory operation happens.
   *
   * NOTE: This field is xLen wide despite returning a virtual address when
   * translating a handle, because this is also how a handle fault returns
   * the encoded handle that faulted. */
  val addr = UInt(xLen.W)
  val miss = Bool()
  /** If the handle is not present or an load/store was attempted with the wrong
   * access permissions on the handle, a handle fault will be raised. The kind
   * of handle fault is given by HTLBExceptions. */
  val handle_fault = new HTLBExceptions
}

/** Class to represented the result of translating an Alaska handle.
 *
 * NOTE: This bundle does NOT track any information about which handle
 * was translated to produce the contained results. */
final class TranslatedHandle(implicit p: Parameters)
  extends BoomBundle()(p) {
  /** The address a handle was translated to. */
  val addr = UInt(maxSVAddrBits.W)
}

final object TranslatedHandle {
  def apply(addr: UInt)(implicit p: Parameters): TranslatedHandle = {
    val th = Wire(new TranslatedHandle())
    require(addr.getWidth == th.addr.getWidth,
      "Provided address must be correct width to be a translated handle")
    th.addr := addr
    th
  }
}

// Events for PerfCounters in the LSU. Nothing in the HTLB reads these back.
class HTLBPerfIO extends Bundle {
  val walk_start = Bool()
  val walk_lane  = UInt(2.W) // the LSU lane whose miss started the walk (memWidth <= 2)
  val fill       = Bool()
}

/** Describes the configuration of an [[HTLB]]'s and its internal components.
 *
 * @param htwCfg The Handle Table Walker ([[HTWConfig]]) configuration.
 * @param htCacheCfg The Handle Table Cache ([[HTCacheConfig]]) configuration.
 */
case class HTLBConfig(
  htwCfg: HTWConfig,
  htCacheCfg: HTCacheConfig,
)

/** Handle Translation Lookaside Buffer.
 *
 * This is an abstract class that describes how all HTLBs interface with other
 * modules.
 *
 * This functions similarly to the translation lookaside buffer (TLB) used for
 * virtual memory. Instead of handling virtual address pages, this operates on
 * and works with handles. HOWEVER, the HTLB receives BOTH handles and normal
 * pointers! You (as the writer of an HTLB implementation) MUST pass those
 * normal address pointers onto the TLB yourself, otherwise your virtual memory
 * translation requests will never be translated.
 *
 */
sealed abstract class HTLB(cfg: HTLBConfig)(implicit p: Parameters)
  extends BoomModule()(p) {
  val io = IO(new Bundle {
    val req = Flipped(Vec(memWidth, Decoupled(new HTLBReq)))
    val resp = Vec(memWidth, new HTLBResp)
    val mem = new HellaCacheIO
    val kill = Input(Bool())
    /** Enable/Disable the HTLB and its state machinery.
     * TODO: What are the effects when you turn off the HTLB midway through
     * operation? */
    val htlb_enabled = Input(Bool())
    /** Is the HTLB in a ready state? */
    val miss_rdy = Output(Bool())
    val perf = Output(new HTLBPerfIO)
    /** The base address of the Handle Table in memory. */
    val htBase = Input(UInt(xLen.W))
    /** The base address where HTLB dumps should be placed in memory. */
    val htDump = Input(UInt(maxSVAddrBits.W))
    /** Handle to be invalidated in the HTLB. */
    val htInval = Input(new HID())
    /** Feedback to handle invalidation CSR that invalidation is complete. */
    val htInvald = Output(Bool())
    /** Raise to true.B to completely empty/clear out the HTLB. */
    val clear_htlb = Input(Bool())
    /** The size of the handle table in memory in bytes. */
    val htSize = Input(UInt(xLen.W))
  })

  /** Convert a handle to a virtual address.
   * Meant to be used for passthrough requests. */
  protected def effectiveAddress(haddr: Handle): UInt = {
    val sum = haddr.asUInt
    val ea_sign = Mux(
      sum(vaddrBits - 1),
      ~sum(63, vaddrBits) === 0.U,
      sum(63, vaddrBits) =/= 0.U
    )

    Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
  }

  assert(implies(io.mem.req.valid,
                 (io.mem.req.bits.addr & ((1.U << io.mem.req.bits.size)) - 1.U) === 0.U),
    "[htlb] HellaCache request from HTLB is misaligned!")

  /* Requirements on HTLB requests. */
  for (w <- 0 until memWidth) {
    /* If the HTLB is on and we get a valid request, check that the requests
     * fields are sane. */
    assert(implies(io.htlb_enabled && io.req(w).valid,
                   implies(!io.req(w).bits.haddr.isHandle(),
                           io.req(w).bits.passthrough)),
      cf"[htlb] Handle translation request for non-handle address ${io.req(w).bits.haddr} must be marked as passthrough")
  }

  /* Requirements on HTLB responses for the given handle translation request. */
  for (w <- 0 until memWidth) {
    /* Is this response for a handle request? */
    val resp_for_handle = io.htlb_enabled && io.req(w).valid &&
      io.req(w).bits.haddr.isHandle() && !io.req(w).bits.passthrough

    /* A handle translation request for something that does not "look like" a
     * handle or is marked as passthrough must NEVER miss. */
    assert(implies(!resp_for_handle, !io.resp(w).miss),
      "[htlb] Non-handle request must response as a \"miss\"")

    /* Only translation requests for handles can produce handle faults. */
    assert(implies(!resp_for_handle,
                   !io.resp(w).handle_fault.ld && !io.resp(w).handle_fault.st),
      "[htlb] Only translations requests for handles can produce handle faults")
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
final class FaultingHTLB(cfg: HTLBConfig)(implicit p: Parameters)
  extends HTLB(cfg)(p) {
  // Memory Request Construction
  val walk_addr = WireInit(0.U(xLen.W))

  /* Make all the connections to the HellaCache L1d$ that the HTW will need. */
  // IO assignments for walker
  io.mem.req.valid := false.B
  io.mem.req.bits.phys := cfg.htwCfg.pht_enabled.B
  io.mem.req.bits.cmd := rocket.M_XRD
  io.mem.req.bits.size := log2Ceil(xBytes).U
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.addr := walk_addr
  io.mem.req.bits.idx.foreach(_ := walk_addr)
  io.mem.req.bits.dprv := { if (cfg.htwCfg.pht_enabled) rocket.PRV.S.U else rocket.PRV.U.U }
  io.mem.req.bits.dv := false.B
  io.mem.req.bits.tag := DontCare
  io.mem.req.bits.no_resp := false.B
  io.mem.req.bits.no_alloc := DontCare
  io.mem.req.bits.no_xcpt := DontCare
  io.mem.req.bits.data := DontCare
  io.mem.req.bits.mask := DontCare
  io.mem.s1_kill := false.B
  io.mem.s1_data.data := 0.U
  io.mem.s1_data.mask := 0.U
  io.mem.s2_kill := false.B
  io.mem.keep_clock_enabled := false.B
  // Uncached response handling (if ever valid)
  io.mem.uncached_resp.foreach(_.ready := true.B)

  io.miss_rdy := true.B
  io.htInvald := false.B

  io.perf.walk_start := false.B
  io.perf.walk_lane  := 0.U
  io.perf.fill       := false.B

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

    val cmd_read  = rocket.isRead(req.bits.cmd)
    val cmd_write = rocket.isWrite(req.bits.cmd)

    /* Is this response for a handle request? */
    val resp_for_handle = io.htlb_enabled && req.valid &&
      req.bits.haddr.isHandle() && !req.bits.passthrough

    /* Despite this module always returning a handle fault, a handle fault is
     * NOT a miss! A miss means that the handle was valid, but that the HTLB
     * has not previously (up to replacement policies) seen that handle and
     * therefore does not have the handle in its HTLB. Because this HTLB will
     * always raise a handle fault, this module can also NEVER miss.
     *
     * If we receive a request for a virtual address and not a handle, that is
     * denoted as a passthrough request. We cannot miss, since by definition
     * the translation request was not for a handle, and we instead just return
     * the virtual address. */
    io.resp(w).miss := false.B

    // These are bit patterns, not UInts, but this is the easiest way to make
    // the type system happy right now.
    val HANDLE_FAULT_LD = "b00".U
    val HANDLE_FAULT_ST = "b01".U
    def handle_fault_haddr_resp(handle: Handle): UInt = {
      val haddr = handle.asUInt
      val fault_haddr = WireInit(0.U(xLen.W))
      // TODO: All commented-out code blocks in here can be re-enabled when
      // there is a boolean controlling if we should even be generating a
      // response, i.e., the resp_for_handle boolean would be passed into here.
      //
      // The ## and Cat methods are the same here. KGH just wanted documentation
      // on 2 ways to achieve the same thing until this gets unified behind some
      // kind of useful abstraction.
      when (cmd_read) {
        fault_haddr := HANDLE_FAULT_LD ## haddr.tail(2)
      } .elsewhen (cmd_write) {
        fault_haddr := Cat(HANDLE_FAULT_ST, haddr(61, 0))
      }//  .otherwise {
      //   assert(false.B, "Handle command was NEITHER load nor store!")
      // }
      require(fault_haddr.getWidth == 64, "Modified faulting haddr must be xLen wide")
      // assert(fault_haddr =/= 0.U, "Impossible handle fault on address 0x0")
      fault_haddr
    }

    io.resp(w).addr := Mux(resp_for_handle,
      handle_fault_haddr_resp(req.bits.haddr),
      effectiveAddress(io.req(w).bits.haddr))

    /* NOTE: If a handle is marked as writable, then this block of hardware
     * will also IMPLICITLY treat it as readable.
     * This is generally the idea, because what good is a handle structure
     * that you can write to that you cannot read from? */
    io.resp(w).handle_fault.ld := resp_for_handle && cmd_read  && (!present || !r_perm)
    io.resp(w).handle_fault.st := resp_for_handle && cmd_write && (!present || !w_perm)
  }
}

/** An invalidation request for the structures that cache handles, the [[HTW]]
 * and [[HTCache]].
 */
final class HandleInvalidation extends Bundle {
  /** The handle to invalidate in Yukon's internal caches. Ignored if [[all]] is
   * true.B. */
  val hid = new HID()
  /** Should every entry be invalidated? If true.B, the value of [[hid]] is
   * ignored. */
  val all = Bool()
}

/** The Yukon HTLB.
  *
  * This HTLB variant *actually* translates handles to virtual addresses in
  * hardware.
  *
  * If a handle comes in, and it misses in YukonHTLB, it replies with the "oops
  * the TLB missed" signal and then does a Handle Table Walk (HTW). The LSU
  * handles this miss response the same way as a missed TLB translation: the
  * LDQ/STQ entry will be retried later.
  * Eventually, the handle will be retried by the LSU at some point again. The
  * handle translation request behaves the same as the first time, but because
  * the Handle Table Walker (HTW) did a lookup in parallel with other requests
  * and behind-the-scenes, this 2nd translation attempt will hit in the HTLB.
  * The HTLB then rewrites the LD/ST request's address, and passes it off to the
  * TLB.
  */
final class YukonHTLB(cfg: HTLBConfig)(implicit p: Parameters)
    extends HTLB(cfg)(p) {

  // The lookup and the walker serve lane 0 only.
  require(memWidth == 1, s"YukonHTLB supports memWidth == 1 only, not $memWidth")

  for (w <- 0 until memWidth) {
    io.resp(w).handle_fault.ld := false.B
    io.resp(w).handle_fault.st := false.B
  }

  /* The HTW & HTCache are in the same clock and reset domain as the HTLB.
   * This means that both modules' internal storage will be reset at the same
   * time as the outer YukonHTLB. So we do not need any special logic for
   * invalidating their contents when reset is triggered. */
  val cache = Module(new HTCache(cfg.htCacheCfg))
  val htw = Module(new HTW(cfg.htwCfg))
  htw.io.htBase := io.htBase
  htw.io.mem <> io.mem
  io.miss_rdy := htw.io.req.ready
  cache.io.fill := htw.io.fill

  io.perf.fill := htw.io.perf.fill

  for (w <- 0 until memWidth) {
    io.resp(w).handle_fault.ld := false.B
    io.resp(w).handle_fault.st := false.B
  }

  val htCache_hit = widthMap(w => cache.io.lookup(w).hit)
  val htCache_way = widthMap(w => cache.io.lookup(w).way)

  val timeline = new TimelineTracker()

  // Utilities. Every LSU lane (memWidth) has its own lookup port. There is one walker.
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hids = widthMap(w => io.req(w).bits.haddr.hid)

  val htlb_miss = widthMap(w => {
    io.req(w).valid && hm_enabled(w) && !htCache_hit(w)
  })

  for (w <- 0 until memWidth) {
    cache.io.lookup(w).en     := hm_enabled(w)
    cache.io.lookup(w).access := io.req(w).valid
    cache.io.lookup(w).hid    := hids(w)
  }

  // Walker arbitration: the lowest lane that misses starts the walk. A miss on
  // another lane in the same cycle reports a miss and retries, as a miss while
  // the walker is busy does.
  val lane_miss = widthMap(w => io.req(w).fire && htlb_miss(w))
  val any_miss  = lane_miss.asUInt.orR
  val walk_lane = if (memWidth == 1) 0.U else PriorityEncoder(lane_miss.asUInt)
  val walk_hid  = hids(walk_lane)

  midas.targetutils.PerfCounter(PopCount(lane_miss), "htlb_miss", "htlb_miss")
  // FIXME: Bring these PerfCounters back
  midas.targetutils.PerfCounter(PopCount(widthMap(w => io.req(w).fire && htCache_hit(w) && hm_enabled(w))), "htlb_hit", "htlb_hit")
  midas.targetutils.PerfCounter(any_miss && htw.io.events.l0_hit_in_s_ready, "htlb_l0_hit", "htlb_l0_hit")

  for (w <- 0 until memWidth) {
    cache.io.lookup(w).en     := hm_enabled(w)
    cache.io.lookup(w).access := io.req(w).valid
    cache.io.lookup(w).hid    := hids(w)
  }

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Response & Output Logic
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  for (w <- 0 until memWidth) {
    val translatedHandle: TranslatedHandle = cache.io.lookup(w).data

    io.req(w).ready := true.B
    io.resp(w).miss := htlb_miss(w)
    io.resp(w).addr := Mux(
      !hm_enabled(w),
      effectiveAddress(io.req(w).bits.haddr),
      Mux(!io.resp(w).miss,
        translatedHandle.addr + io.req(w).bits.haddr.offset.asUInt,
        0.U
      )
    )
  }

  // Invalidation

  // htInval is a CSR that software writes to request an invalidation.
  // The CSR clears itself when hrInvald asserts, so we don't need pulse logic.
  val do_htInval = htw.io.req.ready && !io.htInval.hidZero()
  val inval_all  = io.htInval === HID(Fill(Handle.HANDLE_ID_BITS, 1.B))

  // Prioritize invalidation over handling a miss.
  htw.io.req.valid := any_miss && !do_htInval
  htw.io.req.bits  := walk_hid
  htw.io.htw_enabled := io.htlb_enabled

  htw.io.inval.valid := io.clear_htlb || (do_htInval && inval_all)
  htw.io.inval.bits.all := io.clear_htlb || (do_htInval && inval_all)
  htw.io.inval.bits.hid := DontCare // The HTW cannot invalidate a single entry?

  cache.io.inval.valid := io.clear_htlb || do_htInval
  cache.io.inval.bits.all := io.clear_htlb || inval_all
  // cache.io.inval.bits.all := io.clear_htlb && inval_all
  cache.io.inval.bits.hid := io.htInval

  when(io.clear_htlb) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.clear_htlb: cycle=%d all entries invalidated\n", timeline.cycle)
    )
  }

  when(do_htInval) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.htInval: cycle=%d hid=0x%x\n", timeline.cycle, io.htInval.asUInt)
    )
  }

  io.htInvald := do_htInval

  midas.targetutils.PerfCounter(io.clear_htlb, "htlb_clear", "htlb_clear")
  midas.targetutils.PerfCounter(do_htInval && !inval_all, "htlb_inval_one", "htlb_inval_one")
  midas.targetutils.PerfCounter(do_htInval && inval_all, "htlb_inval_all", "htlb_inval_all")
  midas.targetutils.PerfCounter(!io.htInval.hidZero() && HTWState.waiting(htw.io.events.state),
    "htlb_inval_wait", "htlb_inval_wait")

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // PerfCounters (see thesis todo/01-htlb-counters.md). These only observe state.
  // The description equals the label: thesis src/yukon/bench.py names result columns by description.
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  val walk_start = htw.io.events.state === HTWState.s_ready && io.htlb_enabled && !do_htInval && any_miss

  io.perf.walk_start := walk_start
  io.perf.walk_lane  := walk_lane

  // FIXME: Can these PerfCounters go into the child modules?
  // There seem to be an awful lot of connections that must be made to grab
  // the relevant logics to trigger the PerfCounters.
  locally {
    val hid_set = widthMap(w => HTLBUtils.hidSplit(hids(w), cfg.htCacheCfg.nSets)._2)

    val fill = htw.io.fill.valid
    // The HID being walked. HTW holds it on its input fill line for the duration of the walk?
    // FIXME: Validate the assumption above is true!
    val hid_req = htw.io.fill.bits.hid

    // Misses in the cycle a walk starts that did not start it (a lower lane did).
    val miss_walker_taken = widthMap(w => lane_miss(w) && walk_start && walk_lane =/= w.U)
    val miss_busy = widthMap(w => lane_miss(w) && htw.io.events.state =/= HTWState.s_ready)
    midas.targetutils.PerfCounter(PopCount(widthMap(w => miss_busy(w) && hids(w) === hid_req)), "htlb_miss_busy_same", "htlb_miss_busy_same")
    midas.targetutils.PerfCounter(PopCount(widthMap(w => miss_busy(w) && hids(w) =/= hid_req)), "htlb_miss_busy_other", "htlb_miss_busy_other")
    midas.targetutils.PerfCounter(PopCount(miss_walker_taken), "htlb_miss_walker_taken", "htlb_miss_walker_taken")

    // Collect walk performance numbers
    val walk_cycles = RegInit(0.U(16.W))
    val walk_max    = RegInit(0.U(16.W))
    when(walk_start) {
      walk_cycles := 1.U
    }.elsewhen(HTWState.waiting(htw.io.events.state) && walk_cycles =/= ~0.U(16.W)) {
      walk_cycles := walk_cycles + 1.U
    }
    when(fill && walk_cycles > walk_max) { walk_max := walk_cycles }

    midas.targetutils.PerfCounter(walk_start, "htlb_walk_start", "htlb_walk_start")
    midas.targetutils.PerfCounter.identity(walk_max, "htlb_walk_lat_max", "htlb_walk_lat_max")

    if (boomParams.enableTwoStageHTW) {
      midas.targetutils.PerfCounter(walk_start && !htw.io.events.l0_hit_in_s_ready,
        "htlb_walk_l0_miss", "htlb_walk_l0_miss")
    }

    if (boomParams.enableHTLBSetCounters) {
      for (s <- 0 until cfg.htCacheCfg.nSets) {
        midas.targetutils.PerfCounter(walk_start && HTLBUtils.hidSplit(walk_hid, cfg.htCacheCfg.idxBits)._2 === s.U, s"htlb_walk_set$s", s"htlb_walk_set$s")
      }
    }
  }
}

/** Collect several handle translation helpers into a single namespace.
 *
 * Many of these should have different homes or modifications made elsewhere to
 * remove the need for these helpers in general. This object is essentially a
 * collection of workarounds.
 */
final object HTLBUtils {
  /** Split a HID. */
  // TODO: Just change rocket.Split to split a Bit instead of UInt?
  // Should this be a method on HIDs instead?
  def hidSplit(hid: HID, idx: Int) = {
    val w = hid.id.getWidth
    (hid.id(w-1, idx), hid.id(idx-1, 0))
  }
}
