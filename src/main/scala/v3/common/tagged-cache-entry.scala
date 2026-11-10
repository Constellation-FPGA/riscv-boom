package boom.v3.common

import chisel3._
import chisel3.util._

/** An "abstract" definition of a tagged cache entry with validity.
 *
 * NOTE: This bundle will NOT work with Chisel versions older than 3.5, because
 * of the class definition of the tagBits parameter. */
class TaggedCacheEntry[T <: Data](tagBits: Int, gen: T) extends Bundle {
  val valid = Bool()
  val tag = UInt(tagBits.W)
  val data: T = gen.cloneType

  def hit(tag: UInt): Bool = {
    require(tag.getWidth == tagBits,
      s"Provided tag $tag is ${tag.getWidth} bits wide, but tagBits=$tagBits!")
    valid && this.tag === tag
  }

  def insert(tag: UInt, data: T): Unit = {
    require(tag.getWidth == tagBits,
      s"Provided tag $tag is ${tag.getWidth} bits wide, but tagBits=$tagBits!")
    valid := true.B
    this.tag := tag
    this.data := data
  }

  def invalidate(): Unit = {
    valid := false.B
    tag := 0.U
    data := 0.U.asTypeOf(gen)
  }
}

object TaggedCacheEntry {
  /** Create a [[TaggedCacheEntry]] object as Chisel hardware, invalidate it, and
   * return the hardware object. Suitable for constructing register banks that
   * serve as caches. */
  def apply[T <: Data](tagBits: Int, gen: T): TaggedCacheEntry[T] = {
    val e = Wire(new TaggedCacheEntry(tagBits, gen))
    e.invalidate()
    e
  }
}
