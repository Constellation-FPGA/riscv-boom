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
    val data = Output(new TranslatedHandle())
    // FIXME: Should way & data become a bundle and be "hidden" inside a Valid?
  }

  /** The handle cache lookup that should be performed. */
  val lookup = Vec(memWidth, new HTCacheLookup(cfg))
  val fill = Input(Valid(new HTFill()))
  val inval = Input(Valid(new HandleInvalidation()))
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

  /** A handle table cache entry. */
  final class Entry(tagBits: Int) extends Bundle {
    val tag = UInt(tagBits.W)
    val data = new TranslatedHandle()
    val valid = Bool()

    def hit(tag: UInt) = { valid && this.tag === tag }

    def insert(tag: UInt, translation: TranslatedHandle) = {
      this.tag := tag
      valid := true.B
      data := translation
    }

    def invalidate() = { valid := false.B }
  }

  final object Entry {
    def apply() = {
      val e = Wire(new Entry(cfg.tagBits))
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
   * structure.
   *
   * XXX: entries is shared between ALL lanes (memWidth)! This will be
   * problematic if the memWidth grows too high because we will end up making
   * a very high-port count memory block! */
  val entries = RegInit(VecInit.fill(cfg.nSets, cfg.nWays)(Entry()))
  val plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  /* "used" is set on a hit and cleared on a fill. */
  val used = RegInit(VecInit.fill(cfg.nSets, cfg.nWays)(false.B))

  val (r_tag, r_idx) = HTLBUtils.hidSplit(io.fill.bits.hid, cfg.idxBits)

  val lookup_hid_tag = io.lookup.map(l => HTLBUtils.hidSplit(l.hid, cfg.idxBits)._1)
  val lookup_hid_set = io.lookup.map(l => HTLBUtils.hidSplit(l.hid, cfg.idxBits)._2)

  // Use PLRU to pick way or first invalid
  val repl_way = {
    /* NOTE: The SetAssocLRU does not handle conditions where nWays <= 1. So
     * we must handle it ourselves. */
    if (cfg.nWays > 1) {
      val valid_entries = VecInit(entries(r_idx).map(_.valid))
      Mux(
        valid_entries.asUInt.andR,
        plru.way(r_idx),
        OHToUInt(PriorityEncoderOH(~valid_entries.asUInt))
      )
    }
    else 0.U
  }

  /* Hit Logic (per lane)
   * Hits per lane (memWidth), per set, and per way. Produces a true.B when the
   * HTCache Entry in the specified lane-set-way combination is a hit.
   * NOTE: This produces a Vec(Vec(Bool))*/
  val real_hits = widthMap(w => VecInit(entries(lookup_hid_set(w)).map(io.lookup(w).en && _.hit(lookup_hid_tag(w)))))
  val hit_way = widthMap(w => OHToUInt(real_hits(w).asUInt))

  // Update PLRU on a hit and on a fill. real_hits is one-hot, so use OHToUInt
  // (OH1ToUInt gave way k+1). A fill marks the new entry as recently used, so
  // the next fill in the set does not evict it before the retry. access()
  // applies the updates in order: lane 0, lane 1, ..., then the fill last.
  val plru_hit = widthMap(w => io.lookup(w).access && real_hits(w).asUInt.orR)
  plru.access(
    lookup_hid_set :+ r_idx,
    widthMap(w => Pipe(plru_hit(w), hit_way(w), 0)) :+ Pipe(io.fill.valid, repl_way, 0))

  for (w <- 0 until memWidth) {
    io.lookup(w).hit  := real_hits(w).asUInt.orR
    io.lookup(w).way  := hit_way(w)
    io.lookup(w).data := entries(lookup_hid_set(w))(hit_way(w)).data

    /* Mark an entry in the handle cached as "used" when there was a request
     * has a hit.
     *
     * The fill write is last in the source file, so fills win. */
    when(io.lookup(w).en && io.lookup(w).hit) {
      used(lookup_hid_set(w))(OHToUInt(real_hits(w))) := true.B
    }
  }

  when(io.fill.valid) {
    val translation = io.fill.bits.xlation
    // No victim handling needed, just overwrite
    entries(r_idx)(repl_way).insert(r_tag, translation)

    // A newly inserted cache entry has not been "used" yet.
    used(r_idx)(repl_way) := false.B
  }

  /* Invalidation */
  val (e_tag, e_set) = HTLBUtils.hidSplit(io.inval.bits.hid, cfg.idxBits)
  for (e <- entries(e_set)) {
    when(e_tag === e.tag) {
      e.invalidate()
    }
  }

  when(reset.asBool || (io.inval.valid && io.inval.bits.all)) {
    entries.foreach(set => set.foreach(way => way.invalidate()))
  }

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
