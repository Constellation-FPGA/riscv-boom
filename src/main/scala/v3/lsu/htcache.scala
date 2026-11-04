package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.{SetAssocLRU, Split}

import boom.v3.common._

/** Describes the configuration of an [[HTCache]].
 *
 * @param nSets The number of sets available inside this HTLB.
 * @param nWays The number of ways available inside each set. Each way is identical.
 */
case class HTCacheConfig(
  nSets: Int,
  nWays: Int,
) {
  def idxBits = log2Ceil(nSets)
  def tagBits = Handle.HANDLE_ID_BITS - idxBits
}

final class HTCacheIO(cfg: HTCacheConfig)(implicit p: Parameters)
  extends BoomBundle {

  final class HTCacheLookup(cfg: HTCacheConfig) extends Bundle {
    /** Enable this handle table cache to hit. false.B for passthrough requests. */
    // FIXME: Rename back to passthrough to match HTLB from before.
    val en = Input(Bool())
    /** This lookup is a real access: a hit updates the replacement state. */
    // FIXME: Do we really need this if we have en/passthrough?
    val access = Input(Bool())
    /** The HID that is being looked up in the handle cache. */
    val hid = Input(new HID())
    /** Did the lookup request hit in the cache? */
    val hit = Output(Bool())
    /** The way that had a hit. Meaningless when [[hit]] is false.B. */
    val way = Output(UInt(log2Ceil(cfg.nWays).W))
    /** The hit entry's data. Meaningless when [[hit]] is false.B. */
    // val data = Output(new HTLBEntryData)
    // data is the same as HTFill.addr
    val data = Output(UInt(maxSVAddrBits.W))
    // FIXME: Should way & data become a bundle and be "hidden" inside a Valid?
  }

  /** The handle cache lookup that should be performed. */
  val lookup = Vec(memWidth, new HTCacheLookup(cfg))
  val fill = Input(Valid(new HTFill()))
  val inval = Input(Valid(new HandleInvalidation()))
  // FIXME: used should be simplified?
  val used = Output(Vec(cfg.nSets, Vec(cfg.nWays, Bool())))
  val victim_valid = Output(Bool())
}

/** Handle Table Cache.
 *
 * The handle cache *must* respond in the same cycle as a request comes in.
 *
 * This module is intended to serve as a lookup cache inside of [[HTLB]]s. It is
 * essentially a helper/wrapper module around an inferred cache memory so that
 * the cache lookup structure can be customized independently of the handle
 * walker and the control logic that drives everything.
 */
final class HTCache(cfg: HTCacheConfig)(implicit p: Parameters)
  extends BoomModule()(p) {

  val io = IO(new HTCacheIO(cfg))

  private val idxBits = log2Ceil(cfg.nSets)
  private val tagBits = Handle.HANDLE_ID_BITS - idxBits

  /** The data cached by the [[HTCache]]. */
  // TODO: Rename this class.
  final class HTLBEntryData() extends Bundle() {
    val addr = UInt(maxSVAddrBits.W) // Virtual object base.
  }

  /** A handle table cache entry. */
  final class Entry extends Bundle {
    val tag = UInt(tagBits.W)
    val data = new HTLBEntryData()
    val valid = Bool()

    def hit(tag: UInt) = { valid && this.tag === tag }

    def insert(tag: UInt, entry: HTLBEntryData) = {
      this.tag := tag
      valid := true.B
      data := entry
    }

    def invalidate() = { valid := false.B }
  }

  final object Entry {
    def apply() = {
      val e = Wire(new Entry)
      e.valid := false.B
      e.tag := DontCare
      e.data := DontCare
      e
    }
  }

  /* FIXME: Move all definitions of widthMap out to a common object method.
   * Perhaps LSU.widthMap? */
  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))

  /* NOTE: RegInit is important so that reset signals actually reset the cache
   * structure. */
  val entries = RegInit(VecInit.fill(cfg.nSets, cfg.nWays)(Entry()))
  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  val (r_tag, r_idx) = HTLBUtils.hidSplit(io.fill.bits.hid, idxBits)

  val hid_tag = io.lookup.map(l => HTLBUtils.hidSplit(l.hid, idxBits)._1)
  val hid_set = io.lookup.map(l => HTLBUtils.hidSplit(l.hid, idxBits)._2)

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

  // Hit Logic (per lane)
  // FIXME: Can we remove the asUInt call here?
  val real_hits = widthMap(w => VecInit(entries(hid_set(w)).map(io.lookup(w).en && _.hit(hid_tag(w)))).asUInt)
  // val real_hits = {
  //   def blah(memLane: Int) = io.lookup(memLane).en && _.hit(hid_tag(memLane))
  //   widthMap(w => VecInit(entries(hid_set(w)).map(blah)).asUInt)
  // }
  val hit_way = widthMap(w => OHToUInt(real_hits(w)))

  // Update PLRU on a hit and on a fill. real_hits is one-hot, so use OHToUInt
  // (OH1ToUInt gave way k+1). A fill marks the new entry as recently used, so
  // the next fill in the set does not evict it before the retry. access()
  // applies the updates in order: lane 0, lane 1, ..., then the fill last.
  val plru_hit = widthMap(w => io.lookup(w).access && real_hits(w).orR)
  l1_plru.access(
    hid_set :+ r_idx,
    widthMap(w => Pipe(plru_hit(w), hit_way(w), 0)) :+ Pipe(io.fill.valid, repl_way, 0))

  // No victim handling needed for simplification, just overwrite
  when(io.fill.valid) {
    val entryData = Wire(new HTLBEntryData())
    entryData.addr := io.fill.bits.addr
    entries(r_idx)(repl_way).insert(r_tag, entryData)
  }

  for (w <- 0 until memWidth) {
    io.lookup(w).hit  := real_hits(w).orR
    io.lookup(w).way  := hit_way(w)
    // FIXME: We should return a better class than just UInt
    // This could be HTLBEntryData, if that class is moved to file-level.
    io.lookup(w).data := entries(hid_set(w))(hit_way(w)).data.addr
  }

  /* Invalidation */
  val (e_tag, e_idx) = HTLBUtils.hidSplit(io.inval.bits.hid, idxBits)
  for (e <- entries(e_idx)) {
    when(e_tag === e.tag) {
      e.invalidate()
    }
  }

  // Lost entries. "used" is set on a hit and cleared on a fill. The fill write is last, so a fill wins.
  val used = RegInit(VecInit(Seq.fill(cfg.nSets)(VecInit(Seq.fill(cfg.nWays)(false.B)))))
  // val used = RegInit(VecInit.fill(cfg.nSets, cfg.nWays)(false.B))
  io.used := used
  for (w <- 0 until memWidth) {
    when(io.lookup(w).en && io.lookup(w).hit) {
      used(hid_set(w))(OHToUInt(real_hits(w))) := true.B
    }
  }
  when(io.fill.valid) { used(r_idx)(repl_way) := false.B }

  io.victim_valid := entries(r_idx)(repl_way).valid
  midas.targetutils.PerfCounter(io.fill.valid, "htlb_fill", "htlb_fill")
  midas.targetutils.PerfCounter(io.fill.valid && entries(r_idx)(repl_way).valid,
    "htlb_evict_valid", "htlb_evict_valid")
  midas.targetutils.PerfCounter(io.fill.valid && entries(r_idx)(repl_way).valid && !used(r_idx)(repl_way),
    "htlb_evict_unused", "htlb_evict_unused")

  // if (boomParams.enableTwoStageHTW) {
  //   val l0_idx = getIndex(io.htInval)
  //   val l0_tag = getTag(io.htInval)
  //   val cache = ht_directory_cache.get
  //   when(cache(l0_idx).valid && cache(l0_idx).tag === l0_tag) {
  //     cache(l0_idx).valid := false.B
  //   }
  // }

  val timeline = new TimelineTracker()

  for (s <- 0 until cfg.nSets) {
    for (w <- 0 until cfg.nWays) {
      val e = entries(s)(w)
      timeline.trackState(s"htlb.s${s}w${w}", "invalid", !e.valid, e.tag)
    }
  }
}
