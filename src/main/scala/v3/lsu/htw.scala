package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.util.Split

import boom.v3.common._

/** Describes the configuration of an [[HTW]].
 *
 * @param pht_enabled True if the address in the htBase CSR is physical.
 */
case class HTWConfig(
  pht_enabled: Boolean = true,
)

/** Side-band information about the performance of the HTW's internals.
 *
 * This is primarily used with performance counters and analyses. No IO
 * critical for the HTW to function should be placed in this bundle.
 */
final class HTWPerf extends Bundle {
  /** Outputs true.B if an incoming handle translation request matches an
   * already in-flight and not yet completed handle translation request. */
  val ht0_hit_in_s_ready = Output(Bool())
}

final class HTWIO(cfg: HTWConfig)(implicit p: Parameters) extends BoomBundle {
  /** Incoming request to translate a handle HID to an address. */
  val req = Flipped(Decoupled(new HID()))
  /** A translated handle, to be passed to the requestor to fill something,
   * usually the [[HTCache]] cache. */
  val fill = Output(Valid(new HandleTranslation()))
  /** Incoming request to invalidate a handle HID. */
  val inval = Input(Valid(new HandleInvalidation()))
  /** When true.B, enable the HTW. Otherwise HTW is kept off.
   * This also controls whether or not memory requests are sent off to the rest
   * of the memory subsystem through HellaCache. */
  val htw_enabled = Input(Bool())
  /** The base address of the handle table in memory. */
  val htBase = Input(UInt(xLen.W))
  /** Output-only wire for performance state of the HTW. */
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

  /** A translation response. This is an internal class used to cast the 8-byte
   * 64-bit result from memory into an object with named fields that are easier
   * for us to reason about and work work.
   */
  // Make this private in the future?
  final class XLateResp(implicit p: Parameters) extends BoomBundle()(p) {
    /** This field should be completely ignored. */
    val ignored = UInt((xLen - (new TranslatedHandle).addr.getWidth).W)
    /** The virtual address for the handle table request.
     * This can be an HT0 address from an HT0 lookup, an HT1 address from an HT1
     * lookup, or a handle's final translation. */
    val addr = UInt(maxSVAddrBits.W)
  }

  val io = IO(new HTWIO(cfg))
  io.perf.ht0_hit_in_s_ready := false.B

  /* The zeroth level is indexed by the top 12 bits of a handle ID and the
   * first level is indexed by the remaining bits (18 bits in this case). */
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

  /* The HTW's translation buffer serves two purposes:
   *  1. It is the HTW's working space as it builds up the HID's translation.
   *  2. Subsequent requests for the same HT0 level can have the HT0->HT1
   *     mapping cached, helping reduce the number of requests to memory for HT0
   *     blocks.
   *
   * The HTW can only accept one handle to translate at a time. Because of
   * latencies when going to memory, it makes sense for us to hold onto "hot"
   * HT0->HT1 translations for some time.
   *
   * The HTW uses this translation buffer to build up a handle's translated
   * address as the HTW walks the structure in-memory. Once a handle is fully
   * translated, the translation exits the buffer and is sent elsewhere. */
  private val translation_buffer = RegInit(VecInit.fill(boomParams.HTWCacheSize)(
    TaggedCacheEntry(htwHT1Bits, UInt(maxSVAddrBits.W))
  ))

  when (io.inval.valid && io.inval.bits.all) {
    translation_buffer.foreach(_.invalidate())
  }

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
  // "Typecast" the UInt(64.W) from the HellaCacheResp to a form we prefer.
  val mem_resp_data  = RegNext(io.mem.resp.bits.data, 0.U).asTypeOf(new XLateResp)

  /* The HID that is currently being translated/walked.
   * The register "holds onto" the HID that was requested so that subsequent FSM
   * stages can answer "this is the HID I am currently translating". It is used
   * by the state machine below to pass HID information between translation
   * states/stages.
   *
   * XXX: There is only ever ONE handle being translated by the HTW at a time! */
  val hid_req = Reg(new HID())

  // NOTE: Using .fire is required because that does req.ready && req.valid!
  when (io.req.fire) {
    hid_req := io.req.bits
  }

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
  midas.targetutils.PerfCounter(state === HTWState.s_wait_ht0 && mem_resp_valid && io.mem.req.fire,
    "htlb_fast_path_fired", "htlb_fast_path_fired")

  io.req.ready := state === HTWState.s_ready

  /* The HandleTranslation bundle is the HTW's response. It is only valid when
   * the handle translation is done. When the translation is done, the memory
   * response data is the handle's translated virtual address.*/
  io.fill.valid := translation_done
  io.fill.bits.hid := hid_req
  io.fill.bits.xlation := TranslatedHandle(mem_resp_data.addr)

  switch(state) {
    is(HTWState.s_ready) {
      when (io.req.valid) {
        val walk_hid = io.req.bits

        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.miss: cycle=%d hid=0x%x htBase=0x%x\n",
              timeline.cycle, walk_hid.id, io.htBase)
          )
        }

        /* Use the translation buffer as a caching structure for HT0->HT1
         * mappings. If there is a valid HT0->HT1 mapping, skip the HT0 lookup
         * and issue the HT1 request directly. */
        val ht0_entry = {
          val ht0_idx = getIndex(walk_hid)
          translation_buffer(ht0_idx)
        }
        val ht0_tag = getTag(walk_hid)
        val ht0_hit = ht0_entry.valid && ht0_entry.tag === ht0_tag

        when(ht0_hit) {
          io.perf.ht0_hit_in_s_ready := true.B
          walk_addr := ht0_entry.data + (hidHT1(walk_hid) * 8.U)
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            if (boomParams.enableStateTracing) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.HT1: cycle=%d hid=0x%x addr=0x%x (HT0 hit)\n",
                  timeline.cycle, walk_hid.id, walk_addr)
              )
            }
            next_state := HTWState.s_wait_ht1
          }.otherwise {
            next_state := HTWState.s_request_ht1
          }
        }.otherwise {
          /* If there is not a valid HT0->HT1 mapping, then do the full HT0
           * lookup by using the htBase value and indexing into that to get the
           * HT0 block. */
          walk_addr := io.htBase + hidHT0(walk_hid) * 8.U
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            if (boomParams.enableStateTracing) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.HT0: cycle=%d hid=0x%x addr=0x%x\n",
                  timeline.cycle, walk_hid.id, walk_addr)
              )
            }
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
            printf("HTLB.walk.HT0: cycle=%d hid=0x%x addr=0x%x\n",
              timeline.cycle, hid_req.id, walk_addr)
          )
        }
        next_state := HTWState.s_wait_ht0
      }
    }

    is(HTWState.s_wait_ht0) {
      // Wait for outer walk response
      when(mem_resp_valid) {
        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.HT0_resp: cycle=%d hid=0x%x base=0x%x\n",
              timeline.cycle, hid_req.id, mem_resp_data.addr)
          )
        }

        /* Update translation buffer with the information we have gotten from
         * memory about the translation so far, adding the HT0->HT1 mapping to
         * the translation buffer for later potential use. */
        val fill_idx = getIndex(hid_req)
        translation_buffer(fill_idx).valid := true.B
        translation_buffer(fill_idx).tag := getTag(hid_req)
        translation_buffer(fill_idx).data := mem_resp_data.addr

        /* NOTE: This is is a "fast path" optimization. If the HTW gets a
         * response for the HT0 request, the HTW will attempt to send out a
         * translation request for the leaf handle table entry (in HT1)
         * IN THE SAME CYCLE the HT0 response arrives. */
        walk_addr := mem_resp_data.addr + (hidHT1(hid_req) * 8.U)
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
      assert(translation_buffer(getIndex(hid_req)).valid,
        cf"Translation buffer has invalid HT1 data for HID $hid_req at index ${getIndex(hid_req)}")
      walk_addr := translation_buffer(getIndex(hid_req)).data + (hidHT1(hid_req) * 8.U)

      io.mem.req.valid := true.B
      when(io.mem.req.fire) {
        if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.walk.HT1: cycle=%d hid=0x%x addr=0x%x\n",
              timeline.cycle, hid_req.id, walk_addr)
          )
        }
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
}
