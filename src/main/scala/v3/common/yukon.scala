package boom.v3.common

import chisel3._
import chisel3.util._

/** Type-level representation of an Alaska handle's ID.
*/
final class HID extends Bundle {
  val id = UInt(Handle.HANDLE_ID_BITS.W)

  def =/=(that: HID): Bool = !(this.id === that.id)

  /** Return true.B if this HID is within the bounds of the handle table,
    * based on the provided HTSIZE. Otherwise, return false.B. */
  def inTable(htSize: UInt) = id < htSize
}

object HID {
  def apply(id: UInt) = {
    val hid = Wire(new HID())
    hid.id := id
    hid
  }
}

/** Type-level representation of an Alaska handle's offset.
*/
final class HandleOffset extends Bundle {
  val offset = UInt(Handle.HANDLE_OFFSET_BITS.W)
}

object HandleOffset {
  def apply(offset: UInt) = {
    val hoff = Wire(new HandleOffset())
    hoff.offset := offset
    hoff
  }
}

/** A hardware description of Alaska's handles for use by Yukon.
 *
 * This is defined as a "Value Class". This means the Handle object is
 * elaborated away by the Chisel compiler, only leaving the handle's underlying
 * bits.
 */
final class Handle extends Bundle {
  val handle = Bits(Handle.HANDLE_WIDTH_BITS.W)

  private def split() = {
    /* FIXME: Make these splits dependent on the Object Handle constants. */
    val magic = handle(63, 62)
    val id = handle(61, 32)
    val offset = handle(31, 0)
    // NOTE: Tuples are 1-indexed.
    (magic, HID(id), HandleOffset(offset))
  }

  /** Return true.B if the bits for this handle "look like" a handle.
   * Returns false.B otherwise.
   *
   * NOTE: For any well-formed handle, this _should_ return true.B. This is
   * provided for completeness more than anything. */
  def isHandle(): Bool = Handle.isHandle(this.handle)

  /** Return this handle's Handle ID (HID). */
  def hid: HID = split()._2

  /** This handle's offset within the object pointed to by this handle. */
  def offset: HandleOffset = split()._3

  /** Return true.B if this handle is within the bounds of the handle table,
    * based on the provided HTSIZE. Otherwise, return false.B. */
  def inTable(htSize: UInt) = hid.inTable(htSize)
}

object Handle {
  // private val HANDLE_WIDTH_BITS: Int = xLen
  val HANDLE_WIDTH_BITS: Int = 64
  val HANDLE_WIDTH_BYTES: Int = HANDLE_WIDTH_BITS / 8
  // require(xLen == HANDLE_WIDTH_BITS, "Handles must be xLen wide")

  /** The bit-pattern that handles must have. */
  private val HANDLE_PATTERN: BitPat = {
    // The top bit must be set and the 2nd-most-top bit must be clear.
    BitPat("b10??????????????????????????????????????????????????????????????")
  }
  require(HANDLE_WIDTH_BITS == HANDLE_PATTERN.getWidth,
    s"Handle mask must be same length as xLen, xLen={xLen}")

  private val HANDLE_MAGIC_BITS: Int = 2
  val HANDLE_ID_BITS: Int = 30
  val HANDLE_OFFSET_BITS: Int = 32
  require(HANDLE_WIDTH_BITS == (HANDLE_MAGIC_BITS + HANDLE_ID_BITS + HANDLE_OFFSET_BITS),
    s"All parts of a handle must have the same width as xLen, xLen={xLen}")

  def apply(haddr: UInt): Handle = {
    require(haddr.getWidth == Handle.HANDLE_WIDTH_BITS,
      s"Yukon handles must be HANDLE_WIDTH_BITS=${Handle.HANDLE_WIDTH_BITS} bits wide")

    val handle = Wire(new Handle())
    handle.handle := haddr
    handle
  }

  /** Return true.B if ADDR "looks like" a handle, otherwise return false.B.
    *
    * We say "looks like" because it is possible to construct a UInt that
    * matches the requirements we place on handles but does NOT correspond to
    * any handle. In such a scenario, this function will return true.B despite
    * the provided UInt not actually being a handle.
    *
    * @param addr Virtual address of a handle as a 64-bit number, before handle
    * translation.
    */
  def isHandle(addr: UInt): Bool = {
    assert(addr.getWidth == HANDLE_WIDTH_BITS,
      "Provided address is not wide enough to be a handle.")
    addr === HANDLE_PATTERN
  }

  /** Convert a [[Handle]] to an address.
   *
   * This uses the handle's HID and its offset to produce the final location
   * requested for this handle.
   */
  def toAddr(handle: Handle, htBase: UInt): UInt = {
    toAddr(handle.hid, htBase, handle.offset)
  }

  /** Convert a [[HID]] to an address within the provided handle table. You may
   * optionally provide a [[HandleOffset]] to do field-level indexing within a
   * handle.
   *
   * By if you do not provide a [[HandleOffset]], an offset of 0 will be used,
   * which will produce the base/start of the object at that HID.
   */
  def toAddr(hid: HID, htBase: UInt,
             off: HandleOffset = HandleOffset(0.U)): UInt = {
    htBase + (hid.id * HANDLE_WIDTH_BYTES.U) + off.offset
  }
}
