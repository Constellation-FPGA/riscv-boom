package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.util.Split

import boom.v3.common._

/** A Handle Table Entry
 *
 * TODO: What is this for? It is only used to convert a mem_resp_data to
 * something else in the s_wait_ht_entry state.
 */
final class HTE(implicit p: Parameters) extends BoomBundle()(p) {
  val reserved = UInt((64 - maxSVAddrBits).W)
  val addr = UInt(maxSVAddrBits.W) // Virtual object base; upper bits are ignored.
}

/** TODO: Document this.
 * ???
 */
// TODO: RENAME THIS
final class TopLevelCacheEntry(htwHT1Bits: Int)(implicit p: Parameters)
  extends BoomBundle()(p) {
// final class HTWEntry(implicit p: Parameters) extends BoomBundle()(p) {
  val valid = Bool()
  val tag = UInt(htwHT1Bits.W) // TODO: Restore this field
  val data = UInt(maxSVAddrBits.W)     // Store the base address for inner walks
}

/** The result from a walk through the handle table in memory. */
// FIXME: Should this go in htcache.scala instead?
final class HTFill(implicit p: Parameters) extends BoomBundle()(p) {
  /** The HID that should fill (be inserted into) the handle cache. */
  val hid = new HID()
  /** The address this translated handle maps to. */
  val xlation = new TranslatedHandle()
}

/** IO class meant for tracking performance events from the [[HTCache]]. */
final class HTWPerf extends Bundle {
  /** Raised to true.B when a fill goes out. */
  val fill = Output(Bool())
}

/** Describes the configuration of an [[HTW]].
 *
 * @param pht_enabled True if the address in the htBase CSR is physical.
 */
case class HTWConfig(
  pht_enabled: Boolean = true,
)

// TODO: Rename this to HTWPerf!
/** Side-band information about the state of the HTW and major events within it.
 *
 * This is primarily used with performance counters and analyses. No IO
 * critical for the HTW to function should be placed in this bundle.
 */
// final class HTWPerf extends Bundle {
final class HTWEvents extends Bundle {
  /** Outputs true.B if an incoming handle translation request matches an
   * already in-flight and not yet completed handle translation request. */
  val ht0_hit_in_s_ready = Output(Bool())
}

final class HTWIO(cfg: HTWConfig)(implicit p: Parameters) extends BoomBundle {
  /** Incoming request to translate a handle HID to an address. */
  val req = Flipped(Decoupled(new HID()))
  /** A translated handle, to be passed to the requestor to fill something,
   * usually the [[HTCache]] cache. */
  val fill = Output(Valid(new HTFill()))
  /** Incoming request to invalidate a handle HID. */
  val inval = Input(Valid(new HandleInvalidation()))
  /** When true.B, enable the HTW. Otherwise HTW is kept off.
   * This also controls whether or not memory requests are sent off to the rest
   * of the memory subsystem through HellaCache. */
  val htw_enabled = Input(Bool())
  /** The base address of the handle table in memory. */
  val htBase = Input(UInt(xLen.W))
  /** Output-only wire for the current state of the HTW.
   * FIXME: This should need to be an output from the HTW. It is only used
   * to have PerfCounters in the upper-level [[YukonHTLB]]. */
// TODO: Rename this to perf!
  val events = Output(new HTWEvents())
  val perf = Output(new HTWPerf())
  /** Connection to memory to translate from the handle table in-memory. */
  val mem = new HellaCacheIO
}

/** The HTW takes in a single handle per clock cycle and attempts to translate
  * it to a [[TranslatedHandle]] and outputs it as an [[HTFill]] response.
  *
  * NOTE: This module does NOT depend on memWidth, so there is a SINGLE HTW
  * shared by all memory lanes.
  */
final class HTW(cfg: HTWConfig)(implicit p: Parameters)
  extends BoomModule()(p) {

  /** States for the FSM in the HTW. */
  private object HTWState extends ChiselEnum {
    /** Ready. Waiting for a miss to start the state machine. */
    val s_ready = Value
    /** Read cache, or issue a request for the entry inside the HT0 level of the
     * handle table in-memory. */
    val s_request_ht0 = Value
    /** Wait for the HT0 entry to return. */
    val s_wait_ht0 = Value
    /** Read cache, or issue a request for the handle's translated address from
     * the HT1 level of the handle table in-memory. */
    val s_request_ht1 = Value
    /** Wait for the HT1 entry to return. Refill when done. */
    val s_wait_ht1 = Value

    def waiting(state: HTWState.Type): Bool = state =/= s_ready
  }

  val io = IO(new HTWIO(cfg))
  io.events.ht0_hit_in_s_ready := false.B

  /* The zeroth level is indexed by the top 12 bits of a handle ID and the
   * first level is indexed by the remaining bits (18 bits in this case). */
  /* FIXME: Should these be parameters to the HTW class? */
  private val htwHT0Bits = 12
  private val htwHT1Bits = Handle.HANDLE_ID_BITS - htwHT0Bits

  // Adapted from HTW.scala
  // We assume twoStageHTW is true for this simplified version with 2-stage
  // TODO: hidHT0 & hidHT1 should instances of custom types.
  private def hidHT0(hid: HID) = hid.id.head(htwHT0Bits)
  private def hidHT1(hid: HID) = hid.id(htwHT1Bits-1, 0)
  private def getEntry(hid: HID) = Split(hidHT0(hid), log2Ceil(boomParams.HTWCacheSize))
  private def getTag(hid: HID) = getEntry(hid)._1
  private def getIndex(hid: HID) = getEntry(hid)._2

  /* The translation buffer is the HTW's working space. While the HTW can only
   * accept one handle to translate at a time, because of latencies when going
   * to memory, it makes sense for us to have multiple translations in flight at
   * the same time.
   *
   * The HTW uses this translation buffer to build up a handle's translated
   * address as the HTW walks the structure in-memory. Once a handle is fully
   * translated, the translation exits the buffer and is sent elsewhere. */
  private val translation_buffer = RegInit(
    VecInit.fill(boomParams.HTWCacheSize)(0.U.asTypeOf(new TopLevelCacheEntry(htwHT1Bits)))
  )

  val timeline = new TimelineTracker()

  // Memory Request Construction
  val walk_addr = Wire(UInt(xLen.W))
  walk_addr := DontCare // assignment in switch

  // IO assignments for walker
  io.mem.req.valid := false.B
  io.mem.req.bits.phys := cfg.pht_enabled.B
  io.mem.req.bits.cmd := rocket.M_XRD
  io.mem.req.bits.size := log2Ceil(xBytes).U
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.addr := walk_addr
  io.mem.req.bits.idx.foreach(_ := walk_addr)
  io.mem.req.bits.dprv := { if (cfg.pht_enabled) rocket.PRV.S.U else rocket.PRV.U.U }
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

  // Disable blocks new memory requests, but does not cancel accepted requests.
  // FIXME: Remove this block?
  when(!io.htw_enabled) {
    io.mem.req.valid := false.B
  }

  // XXX: must use RegNext here to break the combinational path
  //      io.mem.resp.valid → io.mem.req.valid that exists in s_wait_ht0.
  //      Without this register, MIDAS's token model deadlocks at cycle 3.
  //      The 1-cycle delay is acceptable given the multi-cycle cache latency.
  val mem_resp_valid = RegNext(io.mem.resp.valid, false.B)
  val mem_resp_data  = RegNext(io.mem.resp.bits.data, 0.U)

  // The requested HID
  val hid_req = Reg(new HID())

  /* State Machine
   * Because the HTLB can be enabled and disabled at runtime, we only let the
   * FSM run when the user has started the HTLB, and therefore the HTW. If the
   * user turns the HTLB off, reset the HTW's FSM to the ready/start state. */
  private val state = RegInit(HTWState.s_ready)
  private val next_state = WireDefault(state)

  state := Mux(io.htw_enabled, next_state, HTWState.s_ready)

  // Raised to true.B when the currently in-flight handle finishes translation.
  private val translation_done = state === HTWState.s_wait_ht1 && mem_resp_valid

  timeline.trackState("htlb.state", "mem_req", io.mem.req.valid)
  timeline.trackState("htlb.state", "mem_resp", io.mem.resp.valid)
  timeline.trackState("htlb.state", "retry_nack", io.mem.s2_nack)
  midas.targetutils.PerfCounter(io.mem.req.fire, "htlb_mem_req", "htlb_mem_req")
  midas.targetutils.PerfCounter(io.mem.s2_nack &&
                                (state === HTWState.s_wait_ht0 || state === HTWState.s_wait_ht1),
    "htlb_mem_nack", "htlb_mem_nack")


  when(!io.htw_enabled && state =/= HTWState.s_ready) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.disabled: cycle=%d state=%d flushed to s_ready\n",
        timeline.cycle, state.asUInt)
    )
  }
  timeline.trackState("htlb.state", "request_ht0", state === HTWState.s_request_ht0)
  timeline.trackState("htlb.state", "wait_ht0", state === HTWState.s_wait_ht0)
  timeline.trackState("htlb.state", "request_ht1", state === HTWState.s_request_ht1)
  timeline.trackState("htlb.state", "wait_ht1", state === HTWState.s_wait_ht1)
  midas.targetutils.PerfCounter(HTWState.waiting(state), "htlb_waiting", "htlb_waiting")
  midas.targetutils.PerfCounter(state === HTWState.s_request_ht0 || state === HTWState.s_request_ht1, "htlb_mem_stall_cycles", "htlb_mem_stall_cycles")
  midas.targetutils.PerfCounter(state === HTWState.s_wait_ht0 || state === HTWState.s_wait_ht1, "htlb_mem_wait_cycles", "htlb_mem_wait_cycles")

  io.req.ready := state === HTWState.s_ready
  // The HTFill bundle is the HTW's response. It is only valid when the handle
  // is done.
  io.fill.valid := translation_done
  io.fill.bits.hid := hid_req

  // NOTE: HTE here is only to "typecast" the UInt(64.W) from the HellaCacheResp
  // to a form we prefer using.
  io.fill.bits.xlation.addr := mem_resp_data.asTypeOf(new HTE).addr

  // io.perf.fill := state === HTWState.s_wait_ht1 && mem_resp_valid
  // perf.fill := state === HTWState.s_wait_ht1 && mem_resp_valid
  io.perf.fill := translation_done
  perf.fill := translation_done

  // Memory Request Construction
  // TODO: What is this register for? We have the translation buffer for this?
  val inner_walk_base = RegInit(0.U(xLen.W))

  switch(state) {
    is(HTWState.s_ready) {
      when (io.req.valid) {
        val walk_hid = io.req.bits
        hid_req := walk_hid

        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.miss: cycle=%d hid=0x%x htBase=0x%x\n",
              timeline.cycle, walk_hid.id, io.htBase)
          )
        }

        val ht0_entry = {
          val ht0_idx = getIndex(walk_hid)
          translation_buffer(ht0_idx)
        }
        val ht0_tag = getTag(walk_hid)
        val ht0_hit = ht0_entry.valid && ht0_entry.tag === ht0_tag

        when(ht0_hit) {
          io.events.ht0_hit_in_s_ready := true.B
          inner_walk_base := ht0_entry.data
          walk_addr := ht0_entry.data + (hidHT1(walk_hid) * 8.U)
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x (HT0 hit)\n",
                timeline.cycle, walk_hid.id, walk_addr)
            )
            next_state := HTWState.s_wait_ht1
          }.otherwise {
            next_state := HTWState.s_request_ht1
          }
        }.otherwise {
          walk_addr := io.htBase + hidHT0(walk_hid) * 8.U
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
                timeline.cycle, walk_hid.id, walk_addr)
            )
            next_state := HTWState.s_wait_ht0
          }.otherwise {
            next_state := HTWState.s_request_ht0
          }
        }
      }
    }

    is(HTWState.s_request_ht0) {
      // Request HT0 entry from memory
      walk_addr := io.htBase + hidHT0(hid_req) * 8.U
      io.mem.req.valid := true.B

      // Only advance state if the request actually fired
      when(io.mem.req.fire) {
        /* TODO: Make this whole if & printf a method inside the HTWState
         * object? The method takes in the NEXT state to go to and modifies
         * the printf appropriately? */
        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
              timeline.cycle, hid_req.id, walk_addr)
          )
        }
        next_state := HTWState.s_wait_ht0
      }
    }

    is(HTWState.s_wait_ht0) {
      // Wait for outer walk response
      when(mem_resp_valid) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.dir_resp: cycle=%d hid=0x%x base=0x%x\n",
            timeline.cycle, hid_req.id, mem_resp_data)
        )

        // Update L0 Cache
        val fill_idx = getIndex(hid_req)
        translation_buffer(fill_idx).valid := true.B
        translation_buffer(fill_idx).tag := getTag(hid_req)
        translation_buffer(fill_idx).data := mem_resp_data

        inner_walk_base := mem_resp_data

        // Fast Path: Request Second Level immediately
        walk_addr := mem_resp_data + (hidHT1(hid_req) * 8.U)
        io.mem.req.valid := true.B

        when (io.mem.req.fire) {
          next_state := HTWState.s_wait_ht1
        } .otherwise {
          next_state := HTWState.s_request_ht1
        }
      }.elsewhen(io.mem.s2_nack) {
        next_state := HTWState.s_request_ht0
      }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
        next_state := HTWState.s_request_ht0
      }
    }

    is(HTWState.s_request_ht1) {
      // Request Inner Entry (HTE)
      walk_addr := inner_walk_base + (hidHT1(hid_req) * 8.U)

      io.mem.req.valid := true.B
      when(io.mem.req.fire) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x\n",
            timeline.cycle, hid_req.id, walk_addr)
        )
        next_state := HTWState.s_wait_ht1
      }
    }

    is(HTWState.s_wait_ht1) {
      when(mem_resp_valid) {
        // The response is taken and passed to the HTCache through the io.fill
        // bundle.
        next_state := HTWState.s_ready
      }.elsewhen(io.mem.s2_nack) {
        next_state := HTWState.s_request_ht1
      }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
        next_state := HTWState.s_request_ht1
      }
    }
  }

  midas.targetutils.PerfCounter(state === HTWState.s_wait_ht0 && mem_resp_valid && io.mem.req.fire,
    "htlb_fast_path_fired", "htlb_fast_path_fired")
}
