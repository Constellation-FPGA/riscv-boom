package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.util.{SetAssocLRU, Split}

import boom.v3.common._

/** A request made to an HTLB.
 *
 * Requires the "handle address" to operate on and the command (CMD) that will
 * be performed.
 */
class HTLBReq(implicit p: Parameters) extends BoomBundle()(p) {
  val haddr = UInt(xLen.W)
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
   * the actual-requested memory operation happens. */
  val addr = UInt(maxSVAddrBits.W)
  val miss = Bool()
  /** If the handle is not present or an load/store was attempted with the wrong
   * access permissions on the handle, a handle fault will be raised. The kind
   * of handle fault is given by HTLBExceptions. */
  val handle_fault = new HTLBExceptions
}

// Events for PerfCounters in the LSU. Nothing in the HTLB reads these back.
class HTLBPerfIO extends Bundle {
  val walk_start = Bool()
  val walk_lane  = UInt(2.W) // the LSU lane whose miss started the walk (memWidth <= 2)
  val fill       = Bool()
}

/** A Handle Table Entry
 */
class HTE(implicit p: Parameters) extends BoomBundle()(p) {
  val reserved = UInt((64 - maxSVAddrBits).W)
  val addr = UInt(maxSVAddrBits.W) // Virtual object base; upper bits are ignored.
}

/** TODO: Document this.
 * ???
 */
class TopLevelCacheEntry(implicit p: Parameters) extends BoomBundle()(p) {
  val valid = Bool()
  val tag = UInt(htwL1Bits.W)
  val data = UInt(maxSVAddrBits.W)     // Store the base address for inner walks
}

/** Describes the configuration of an [[HTLB]].
 *
 * @param nSets The number of sets available inside this HTLB.
 * @param nWays The number of ways available inside each set. Each way is identical.
 */
case class HTLBConfig(
  nSets: Int,
  nWays: Int,
  pht_enabled: Boolean = true,
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
abstract class HTLB(cfg: HTLBConfig)(implicit p: Parameters)
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
    val htInval = Input(UInt(handleBits.W))
    /** Feedback to handle invalidation CSR that invalidation is complete. */
    val htInvald = Output(Bool())
    /** Raise to true.B to completely empty/clear out the HTLB. */
    val clear_htlb = Input(Bool())
    /** The size of the handle table in memory in bytes. */
    val htSize = Input(UInt(xLen.W))
  })

  assert(implies(io.mem.req.valid,
                 (io.mem.req.bits.addr & ((1.U << io.mem.req.bits.size)) - 1.U) === 0.U),
    "[htlb] HellaCache request from HTLB is misaligned!")

  /* Requirements on HTLB requests. */
  for (w <- 0 until memWidth) {
    /* If the HTLB is on and we get a valid request, check that the requests
     * fields are sane. */
    assert(implies(io.htlb_enabled && io.req(w).valid,
                   implies(!is_handle(io.req(w).bits.haddr),
                           io.req(w).bits.passthrough)),
      cf"[htlb] Handle translation request for non-handle address ${io.req(w).bits.haddr} must be marked as passthrough")
  }

  /* Requirements on HTLB responses for the given handle translation request. */
  for (w <- 0 until memWidth) {
    /* Is this response for a handle request? */
    val resp_for_handle = io.htlb_enabled && io.req(w).valid &&
      is_handle(io.req(w).bits.haddr) && !io.req(w).bits.passthrough

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
class FaultingHTLB(cfg: HTLBConfig)(implicit p: Parameters) extends HTLB(cfg)(p) {
  // Memory Request Construction
  val walk_addr = WireInit(0.U(xLen.W))

  /* Make all the connections to the HellaCache L1d$ that the HTW will need. */
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
      is_handle(req.bits.haddr) && !req.bits.passthrough

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

    val effective_address = {
      val sum = io.req(w).bits.haddr
      val ea_sign = Mux(
        sum(vaddrBits - 1),
        ~sum(63, vaddrBits) === 0.U,
        sum(63, vaddrBits) =/= 0.U
      )

      Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
    }

    io.resp(w).addr := effective_address

    /* NOTE: If a handle is marked as writable, then this block of hardware
     * will also IMPLICITLY treat it as readable.
     * This is generally the idea, because what good is a handle structure
     * that you can write to that you cannot read from? */
    io.resp(w).handle_fault.ld := resp_for_handle && cmd_read  && (!present || !r_perm)
    io.resp(w).handle_fault.st := resp_for_handle && cmd_write && (!present || !w_perm)
  }
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
class YukonHTLB(cfg: HTLBConfig)(implicit p: Parameters)
    extends HTLB(cfg)(p) {
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Data Structures
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  // The lookup and the walker serve lane 0 only.
  require(memWidth == 1, s"HTLBSimple supports memWidth == 1 only, not $memWidth")

  class HTLBEntryData() extends Bundle() {
    val addr = UInt(maxSVAddrBits.W) // Virtual object base.
  }

  class Entry(nSets: Int) extends Bundle {
    val idxBits = log2Ceil(nSets)
    val tagBits = handleBits - idxBits
    val tag = UInt(tagBits.W)
    val data = UInt(new HTLBEntryData().getWidth.W)
    val valid = Bool()

    def hit(tag: UInt) = {
      valid && this.tag === tag
    }

    def insert(tag: UInt, entry: HTLBEntryData) = {
      this.tag := tag

      valid := true.B
      data := entry.asUInt
    }

    def invalidate() = { valid := false.B }
  }

  for (w <- 0 until memWidth) {
    io.resp(w).handle_fault.ld := false.B
    io.resp(w).handle_fault.st := false.B
  }

  val twoStageHTW = boomParams.enableTwoStageHTW

  val timeline = new TimelineTracker()

  // Utilities. Every LSU lane (memWidth) has its own lookup port. There is one walker.
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))
  val hm_enabled = widthMap(w => !io.req(w).bits.passthrough)
  val hids = widthMap(w => hid(io.req(w).bits.haddr))
  val idxBits = log2Ceil(cfg.nSets)

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  // Hit Logic (per lane)
  val hid_tag = widthMap(w => Split(hids(w), idxBits)._1)
  val hid_set = widthMap(w => Split(hids(w), idxBits)._2)
  val real_hits = widthMap(w => VecInit(entries(hid_set(w)).map(hm_enabled(w) && _.hit(hid_tag(w)))).asUInt)
  val htlb_hit = widthMap(w => real_hits(w).orR)
  val htlb_miss = widthMap(w => hm_enabled(w) && !htlb_hit(w))

  // Walker arbitration: the lowest lane that misses starts the walk. A miss on
  // another lane in the same cycle reports a miss and retries, as a miss while
  // the walker is busy does.
  val lane_miss = widthMap(w => io.req(w).fire && htlb_miss(w))
  val any_miss  = lane_miss.asUInt.orR
  val walk_lane = if (memWidth == 1) 0.U else PriorityEncoder(lane_miss.asUInt)
  val walk_hid  = hids(walk_lane)

  // The PLRU is updated after the fill target (r_idx, repl_way) is known, below.

  // The requested HID
  val hid_req = Reg(UInt(handleBits.W))

  // State Machine
  // - ready: idle, waiting for a miss.
  // - request_ht_directory: read cache, or issue a request for the HT directory entry
  // - wait_ht_directory: wait for the HT directory entry to return
  // - request_ht_entry: read cache, or issue a request for the HT entry
  // - wait_ht_entry: wait for the HT entry to return. Refill when done.
  val s_ready :: s_request_ht_directory :: s_wait_ht_directory :: s_request_ht_entry :: s_wait_ht_entry :: Nil = Enum(5)
  val state = RegInit(s_ready)
  val next_state = WireDefault(state)





  // XXX: must use RegNext here to break the combinational path
  //      io.mem.resp.valid → io.mem.req.valid that exists in s_wait_ht_directory.
  //      Without this register, MIDAS's token model deadlocks at cycle 3.
  //      The 1-cycle delay is acceptable given the multi-cycle cache latency.
  val mem_resp_valid = RegNext(io.mem.resp.valid, false.B)
  val mem_resp_data  = RegNext(io.mem.resp.bits.data, 0.U)

  // HTW: L0 Cache (Top Level Cache) Setup
  // Adapted from HTW.scala
  // We assume twoStageHTW is true for this simplified version with 2-stage
  def getEntry(hid: UInt) = {
    val hidL0 = hid.head(htwL0Bits)
    val splitL0TagIndexBits = log2Ceil(boomParams.HTWCacheSize)
    Split(hidL0, log2Ceil(boomParams.HTWCacheSize))
  }
  def getTag(hid: UInt) = getEntry(hid)._1
  def getIndex(hid: UInt) = getEntry(hid)._2
  val ht_directory_cache = {
    if (twoStageHTW)
      Some(
        RegInit(
          VecInit(
            Seq.fill(boomParams.HTWCacheSize)(0.U.asTypeOf(new TopLevelCacheEntry))
          )
        )
      )
    else None
  }

  // Hoisted wire so htlb_l0_hit PerfCounter can reference it outside the s_ready block
  val l0_hit_in_s_ready = WireDefault(false.B)

  val inner_walk_base = RegInit(0.U(xLen.W))

  // htInval is a CSR that software writes to request an invalidation.
  // The CSR clears itself when hrInvald asserts, so we don't need pulse logic.
  val do_htInval = (state === s_ready) && io.htInval.orR
  val inval_all  = io.htInval === Fill(handleBits, 1.B)

  // Memory Request Construction
  val walk_addr = Wire(UInt(xLen.W))
  walk_addr := 0.U // assignment in switch

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

  // FSM Implementation
  state := Mux(io.htlb_enabled, next_state, s_ready)
  when(!io.htlb_enabled && state =/= s_ready) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.disabled: cycle=%d state=%d flushed to s_ready\n",
        timeline.cycle, state)
    )
  }

  timeline.trackState("htlb.state", "request_ht_directory", state === s_request_ht_directory)
  timeline.trackState("htlb.state", "wait_ht_directory", state === s_wait_ht_directory)
  timeline.trackState("htlb.state", "request_ht_entry", state === s_request_ht_entry)
  timeline.trackState("htlb.state", "wait_ht_entry", state === s_wait_ht_entry)

  timeline.trackState("htlb.state", "mem_req", io.mem.req.valid)
  timeline.trackState("htlb.state", "mem_resp", io.mem.resp.valid)
  timeline.trackState("htlb.state", "retry_nack", io.mem.s2_nack)


  // Consider the number of times the HTLB is not ready (that is, it is dealing with a miss, blocking the LSU)
  midas.targetutils.PerfCounter(state =/= s_ready, "htlb_waiting", "htlb_waiting")
  midas.targetutils.PerfCounter(PopCount(lane_miss), "htlb_miss", "htlb_miss")
  midas.targetutils.PerfCounter(PopCount(widthMap(w => io.req(w).fire && htlb_hit(w) && hm_enabled(w))), "htlb_hit", "htlb_hit")
  midas.targetutils.PerfCounter(state === s_request_ht_directory || state === s_request_ht_entry, "htlb_mem_stall_cycles", "htlb_mem_stall_cycles")
  midas.targetutils.PerfCounter(any_miss && l0_hit_in_s_ready, "htlb_l0_hit", "htlb_l0_hit")
  midas.targetutils.PerfCounter(state === s_wait_ht_directory || state === s_wait_ht_entry, "htlb_mem_wait_cycles", "htlb_mem_wait_cycles")

  for (s <- 0 until cfg.nSets) {
    for (w <- 0 until cfg.nWays) {
      val e = entries(s)(w)
      timeline.trackState(s"htlb.s${s}w${w}", "invalid", !e.valid, e.tag)
    }
  }


  // Fill target. Hoisted out of s_wait_ht_entry so the PerfCounters can see the victim.
  val (r_tag, r_idx) = Split(hid_req, idxBits)
  // Use PLRU to pick way or first invalid
  val candidate_repl_way =
    (if (cfg.nWays > 1) l1_plru.way(r_idx) else 0.U)
  val repl_way =
    if (cfg.nWays > 1) {
      val valid_entries = VecInit(entries(r_idx).map(_.valid))
      Mux(
        valid_entries.asUInt.andR,
        candidate_repl_way,
        OHToUInt(PriorityEncoderOH(~valid_entries.asUInt))
      )
    }
    else 0.U

  // Update PLRU on a hit and on a fill. real_hits is one-hot, so use OHToUInt
  // (OH1ToUInt gave way k+1). A fill marks the new entry as recently used, so
  // the next fill in the set does not evict it before the retry. access()
  // applies the updates in order: lane 0, lane 1, ..., then the fill last.
  val plru_hit  = widthMap(w => io.req(w).valid && hm_enabled(w) && real_hits(w).orR)
  val plru_fill = state === s_wait_ht_entry && mem_resp_valid
  l1_plru.access(
    hid_set :+ r_idx,
    (0 until memWidth).map(w => Pipe(plru_hit(w), OHToUInt(real_hits(w)), 0)) :+ Pipe(plru_fill, repl_way, 0))

  switch(state) {



    is(s_ready) {
      // Prioritize invalidation over handling a miss
      when(do_htInval) {
      } .elsewhen(any_miss) {
        hid_req := walk_hid

        // if (boomParams.enableStateTracing) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.miss: cycle=%d hid=0x%x htBase=0x%x\n",
              timeline.cycle, walk_hid, io.htBase)
          )
        // }

        if (twoStageHTW) {
          val cache = ht_directory_cache.get
          val l0_idx = getIndex(walk_hid)
          val l0_tag = getTag(walk_hid)
          val l0_entry = cache(l0_idx)
          val l0_hit = l0_entry.valid && l0_entry.tag === l0_tag

          when(l0_hit) {
            l0_hit_in_s_ready := true.B
            inner_walk_base := l0_entry.data
            walk_addr := l0_entry.data + (walk_hid(htwL1Bits - 1, 0)) * 8.U
            io.mem.req.valid := true.B
            when(io.mem.req.fire) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x (L0 hit)\n",
                  timeline.cycle, walk_hid, walk_addr)
              )
              next_state := s_wait_ht_entry
            }.otherwise {
              next_state := s_request_ht_entry
            }
          }.otherwise {
            walk_addr := io.htBase + (walk_hid >> htwL1Bits) * 8.U
            io.mem.req.valid := true.B
            when(io.mem.req.fire) {
              midas.targetutils.SynthesizePrintf(
                printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
                  timeline.cycle, walk_hid, walk_addr)
              )
              next_state := s_wait_ht_directory
            }.otherwise {
              next_state := s_request_ht_directory
            }
          }
        } else {
          walk_addr := io.htBase + walk_hid * 8.U
          io.mem.req.valid := true.B
          when(io.mem.req.fire) {
            next_state := s_wait_ht_entry
          }.otherwise {
            next_state := s_request_ht_entry
          }
        }
      }
    }


    is(s_request_ht_directory) {
      if (twoStageHTW) {
        // Request L0 entry from memory
        // Addr = htBase + (hid >> 18) * 8
        walk_addr := io.htBase + (hid_req >> htwL1Bits) * 8.U
        io.mem.req.valid := true.B

        // Only advance state if the request actually fired
        when(io.mem.req.fire) {
          // if (boomParams.enableStateTracing) {
            midas.targetutils.SynthesizePrintf(
              printf("HTLB.walk.dir: cycle=%d hid=0x%x addr=0x%x\n",
                timeline.cycle, hid_req, walk_addr)
            )
          // }
          next_state := s_wait_ht_directory
        }
      }
    }



    is(s_wait_ht_directory) {
      if (twoStageHTW) {
        // Wait for outer walk response
        when(mem_resp_valid) {
          midas.targetutils.SynthesizePrintf(
            printf("HTLB.dir_resp: cycle=%d hid=0x%x base=0x%x\n",
              timeline.cycle, hid_req, mem_resp_data)
          )

          // Update L0 Cache
          val cache = ht_directory_cache.get
          val fill_idx = getIndex(hid_req)
          cache(fill_idx).valid := true.B
          cache(fill_idx).tag := getTag(hid_req)
          cache(fill_idx).data := mem_resp_data

          inner_walk_base := mem_resp_data

          // Fast Path: Request Second Level immediately
          // Addr = mem_resp_data + (hid_req & mask) * 8
          walk_addr := mem_resp_data + (hid_req(htwL1Bits - 1, 0)) * 8.U
          io.mem.req.valid := true.B

          when (io.mem.req.fire) {
            next_state := s_wait_ht_entry
          } .otherwise {
            next_state := s_request_ht_entry
          }

        }.elsewhen(io.mem.s2_nack) {
          next_state := s_request_ht_directory
        }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
          next_state := s_request_ht_directory
        }
      }
    }



    is(s_request_ht_entry) {
      // Request Inner Entry (HTE)
      // Addr = inner_walk_base + (hid & mask) * 8
      // mask for 18 bits = (1 << 18) - 1
      if (twoStageHTW) {
        walk_addr := inner_walk_base + (hid_req(
          htwL1Bits - 1,
          0
        )) * 8.U
      } else {
        walk_addr := io.htBase + hid_req * 8.U
      }

      io.mem.req.valid := true.B

      when(io.mem.req.fire) {
        midas.targetutils.SynthesizePrintf(
          printf("HTLB.walk.entry: cycle=%d hid=0x%x addr=0x%x\n",
            timeline.cycle, hid_req, walk_addr)
        )
        next_state := s_wait_ht_entry
      }
    }



    is(s_wait_ht_entry) {
      when(mem_resp_valid) {
        // Insert into L1 entries at valid hid_req
        // No victim handling needed for simplificaiton, just overwrite
        val newEntry = Wire(new HTLBEntryData)
        val hte = mem_resp_data.asTypeOf(new HTE)
        newEntry.addr := hte.addr

        entries(r_idx)(repl_way).insert(r_tag, newEntry)

        next_state := s_ready

      }.elsewhen(io.mem.s2_nack) {
        next_state := s_request_ht_entry
      }.elsewhen(io.mem.s2_xcpt.asUInt.orR) {
        next_state := s_request_ht_entry
      }
    }



  }

  // Disable blocks new memory requests, but does not cancel accepted requests.
  when(!io.htlb_enabled) {
    io.mem.req.valid := false.B
  }

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Response & Output Logic
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  io.miss_rdy := state === s_ready

  for (w <- 0 until memWidth) {
    val entry_data =
      entries(hid_set(w))(OHToUInt(real_hits(w))).data.asTypeOf(new HTLBEntryData)
    val addr = entry_data.addr

    val effective_address = {
      val sum = io.req(w).bits.haddr
      val ea_sign = Mux(
        sum(vaddrBits - 1),
        ~sum(63, vaddrBits) === 0.U,
        sum(63, vaddrBits) =/= 0.U
      )

      Cat(ea_sign, sum(vaddrBits - 1, 0)).asUInt
    }

    io.req(w).ready := true.B
    io.resp(w).miss := htlb_miss(w)

    io.resp(w).addr := Mux(
      !hm_enabled(w),
      effective_address,
      Mux(!io.resp(w).miss,
        entry_data.addr + handle_offset(io.req(w).bits.haddr),
        0.U
      )
    )
  }

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // Tie-offs and unused IO
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  // Reset
  when(reset.asBool || io.clear_htlb) {
    entries.foreach(_.foreach(_.invalidate()))
    ht_directory_cache.foreach(_.foreach(_.valid := false.B))
    when(io.clear_htlb) {
      midas.targetutils.SynthesizePrintf(
        printf("HTLB.clear_htlb: cycle=%d all entries invalidated\n", timeline.cycle)
      )
    }
  }

  // Invalidation
  when(do_htInval) {
    midas.targetutils.SynthesizePrintf(
      printf("HTLB.htInval: cycle=%d hid=0x%x\n", timeline.cycle, io.htInval)
    )
    // Invalidate L1
    when (inval_all) {
      entries.foreach(_.foreach(_.invalidate()))
      ht_directory_cache.foreach(_.foreach(_.valid := false.B))
    }.otherwise {
      val (e_tag, e_idx) = Split(io.htInval, idxBits)
      for (e <- entries(e_idx)) {
        when(e_tag === e.tag) {
          e.invalidate()
        }
      }

      // if (twoStageHTW) {
      //   val l0_idx = getIndex(io.htInval)
      //   val l0_tag = getTag(io.htInval)
      //   val cache = ht_directory_cache.get
      //   when(cache(l0_idx).valid && cache(l0_idx).tag === l0_tag) {
      //     cache(l0_idx).valid := false.B
      //   }
      // }

    }
  }

  io.htInvald := do_htInval

  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------
  // PerfCounters (see thesis todo/01-htlb-counters.md). These only observe state.
  // The description equals the label: thesis src/yukon/bench.py names result columns by description.
  // ------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------

  val walk_start = state === s_ready && io.htlb_enabled && !do_htInval && any_miss
  val fill       = state === s_wait_ht_entry && mem_resp_valid
  val waiting    = state =/= s_ready

  io.perf.walk_start := walk_start
  io.perf.walk_lane  := walk_lane
  io.perf.fill       := fill

  // Walks
  val walk_cycles = RegInit(0.U(16.W))
  val walk_max    = RegInit(0.U(16.W))
  when(walk_start) {
    walk_cycles := 1.U
  }.elsewhen(waiting && walk_cycles =/= ~0.U(16.W)) {
    walk_cycles := walk_cycles + 1.U
  }
  when(fill && walk_cycles > walk_max) { walk_max := walk_cycles }

  midas.targetutils.PerfCounter(walk_start, "htlb_walk_start", "htlb_walk_start")
  if (twoStageHTW) {
    midas.targetutils.PerfCounter(walk_start && !l0_hit_in_s_ready, "htlb_walk_l0_miss", "htlb_walk_l0_miss")
    midas.targetutils.PerfCounter(state === s_wait_ht_directory && mem_resp_valid && io.mem.req.fire,
      "htlb_fast_path_fired", "htlb_fast_path_fired")
  }
  midas.targetutils.PerfCounter(fill, "htlb_fill", "htlb_fill")
  midas.targetutils.PerfCounter.identity(walk_max, "htlb_walk_lat_max", "htlb_walk_lat_max")
  midas.targetutils.PerfCounter(io.mem.req.fire, "htlb_mem_req", "htlb_mem_req")
  midas.targetutils.PerfCounter(io.mem.s2_nack && (state === s_wait_ht_directory || state === s_wait_ht_entry),
    "htlb_mem_nack", "htlb_mem_nack")

  // Lost entries. "used" is set on a hit and cleared on a fill. The fill write is last, so a fill wins.
  val used = RegInit(VecInit(Seq.fill(cfg.nSets)(VecInit(Seq.fill(cfg.nWays)(false.B)))))
  for (w <- 0 until memWidth) {
    when(io.req(w).valid && htlb_hit(w)) { used(hid_set(w))(OHToUInt(real_hits(w))) := true.B }
  }
  when(fill) { used(r_idx)(repl_way) := false.B }

  val victim_valid = entries(r_idx)(repl_way).valid
  val victim_used  = used(r_idx)(repl_way)
  val miss_busy    = widthMap(w => lane_miss(w) && waiting)
  // Misses in the cycle a walk starts that did not start it (a lower lane did).
  val miss_walker_taken = widthMap(w => lane_miss(w) && walk_start && walk_lane =/= w.U)

  midas.targetutils.PerfCounter(fill && victim_valid, "htlb_evict_valid", "htlb_evict_valid")
  midas.targetutils.PerfCounter(fill && victim_valid && !victim_used, "htlb_evict_unused", "htlb_evict_unused")
  midas.targetutils.PerfCounter(io.clear_htlb, "htlb_clear", "htlb_clear")
  midas.targetutils.PerfCounter(do_htInval && !inval_all, "htlb_inval_one", "htlb_inval_one")
  midas.targetutils.PerfCounter(do_htInval && inval_all, "htlb_inval_all", "htlb_inval_all")
  midas.targetutils.PerfCounter(io.htInval.orR && waiting, "htlb_inval_wait", "htlb_inval_wait")
  midas.targetutils.PerfCounter(PopCount(widthMap(w => miss_busy(w) && hids(w) === hid_req)), "htlb_miss_busy_same", "htlb_miss_busy_same")
  midas.targetutils.PerfCounter(PopCount(widthMap(w => miss_busy(w) && hids(w) =/= hid_req)), "htlb_miss_busy_other", "htlb_miss_busy_other")
  midas.targetutils.PerfCounter(PopCount(miss_walker_taken), "htlb_miss_walker_taken", "htlb_miss_walker_taken")

  if (boomParams.enableHTLBSetCounters) {
    for (s <- 0 until cfg.nSets) {
      midas.targetutils.PerfCounter(walk_start && Split(walk_hid, idxBits)._2 === s.U, s"htlb_walk_set$s", s"htlb_walk_set$s")
    }
  }
}
