package boom.v3.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.{SetAssocLRU}

import boom.v3.common._

/** A Handle Table Entry
 */
final class HTE(implicit p: Parameters) extends BoomBundle()(p) {
  val reserved = UInt((64 - maxSVAddrBits).W)
  val addr = UInt(maxSVAddrBits.W) // Virtual object base; upper bits are ignored.
}

/** TODO: Document this.
 * ???
 */
final class TopLevelCacheEntry(implicit p: Parameters) extends BoomBundle()(p) {
  val valid = Bool()
  val tag = UInt(htwL1Bits.W)
  val data = UInt(maxSVAddrBits.W)     // Store the base address for inner walks
}

/** Describes the configuration of an [[HTCache]].
 *
 * @param nSets The number of sets available inside this HTLB.
 * @param nWays The number of ways available inside each set. Each way is identical.
 */
case class HTCacheConfig(
  nSets: Int,
  nWays: Int,
)

/** Handle Table Cache.
 *
 * This module is intended to serve as a lookup cache inside of [[HTLB]]s.
 */
final class HTCache(cfg: HTCacheConfig)(implicit p: Parameters)
  extends BoomModule()(p) {

  class HTLBEntryData() extends Bundle() {
    val addr = UInt(maxSVAddrBits.W) // Virtual object base.
  }

  class Entry(nSets: Int) extends Bundle {
    private val idxBits = log2Ceil(nSets)
    private val tagBits = handleBits - idxBits

    val tag = UInt(tagBits.W)
    val data = UInt(new HTLBEntryData().getWidth.W)
    val valid = Bool()

    def hit(tag: UInt) = { valid && this.tag === tag }

    def insert(tag: UInt, entry: HTLBEntryData) = {
      this.tag := tag
      valid := true.B
      data := entry.asUInt
    }

    def invalidate() = { valid := false.B }
  }

  val timeline = new TimelineTracker()

  // L1 TLB Entries
  val entries = Reg(Vec(cfg.nSets, Vec(cfg.nWays, new Entry(cfg.nSets))))

  val l1_plru = new SetAssocLRU(cfg.nSets, cfg.nWays, "plru")

  // TODO: Just change rocket.Split to split a Bit instead of UInt?
  private def hidSplit(hid: HID, idx: Int) = {
    val w = hid.id.getWidth
    (hid.id(w-1, idx), hid.id(idx-1, 0))
  }
  val hid_tag = widthMap(w => hidSplit(hids(w), idxBits)._1)
  val hid_set = widthMap(w => hidSplit(hids(w), idxBits)._2)

  for (s <- 0 until cfg.nSets) {
    for (w <- 0 until cfg.nWays) {
      val e = entries(s)(w)
      timeline.trackState(s"htlb.s${s}w${w}", "invalid", !e.valid, e.tag)
    }
  }



}
