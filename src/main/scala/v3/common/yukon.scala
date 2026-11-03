package boom.v3.common

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

/** Type-level representation of an Alaska handle's ID.
*/
final class HID extends Bundle {
  val id = UInt(Handle.HANDLE_ID_BITS.W)

  def ===(that: HID): Bool = this.id === that.id
  def =/=(that: HID): Bool = !(this.id === that.id)

  /** Return true.B if this HID is within the bounds of the handle table,
    * based on the provided HTSIZE. Otherwise, return false.B. */
  def inTable(htSize: UInt) = id < htSize

  /** Return true.B if this HID is the "zero HID", the handle with ID 0. */
  def hidZero(): Bool = this.id === 0.U
}

object HID {
  def apply(id: UInt) = {
    require(id.getWidth == Handle.HANDLE_ID_BITS,
      "New HIDs must be constructed with the right number of bits")
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

  /** Return true.B if these bits are a control handle. See
   * [[Handle.isControl]]. */
  def isControl(): Bool = Handle.isControl(this.handle)

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

  private val HANDLE_MAGIC_BITS: Int = 2
  val HANDLE_ID_BITS: Int = 30
  val HANDLE_OFFSET_BITS: Int = 32

  require(HANDLE_WIDTH_BITS == (HANDLE_MAGIC_BITS + HANDLE_ID_BITS + HANDLE_OFFSET_BITS),
    s"All parts of a handle must have the same width as xLen, xLen={xLen}")

  /** The "magic" bit pattern that addresses must have to be considered a
    * handle. */
  private val HANDLE_MAGIC = BitPat("b10")

  /** The bit-pattern that handles must have. */
  private val HANDLE_PATTERN: BitPat = {
    val hidPat = BitPat("b" + "?" * HANDLE_ID_BITS)
    val offPat = BitPat("b" + "?" * HANDLE_OFFSET_BITS)
    HANDLE_MAGIC ## hidPat ## offPat
  }
  require(HANDLE_WIDTH_BITS == HANDLE_PATTERN.getWidth,
    s"Handle mask must be same length as xLen, xLen={xLen}")

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
    require(addr.getWidth == HANDLE_WIDTH_BITS,
      "Provided address is not wide enough to be a handle.")
    addr === HANDLE_PATTERN
  }

  /** The bit-pattern of a control handle: an access to the HTLB itself. */
  private val CONTROL_PATTERN: BitPat = {
    val hidPat = BitPat("b" + "?" * HANDLE_ID_BITS)
    val offPat = BitPat("b" + "?" * HANDLE_OFFSET_BITS)
    BitPat("b11") ## hidPat ## offPat
  }
  require(HANDLE_WIDTH_BITS == CONTROL_PATTERN.getWidth,
    s"Control handle mask must be same length as xLen, xLen={xLen}")

  /** Return true.B if ADDR has the control handle pattern (top bits 11).
    *
    * A control handle has the same fields as a handle. The offset field
    * selects an HTLB control register: register R is at offset R * 8.
    * This is only meaningful in U-mode with the HTLB on. Elsewhere, top
    * bits 11 are a normal (supervisor) virtual address.
    */
  def isControl(addr: UInt): Bool = {
    require(addr.getWidth == HANDLE_WIDTH_BITS,
      "Provided address is not wide enough to be a handle.")
    addr === CONTROL_PATTERN
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

/** An enumeration to describe the kinds of handle exceptions that can be
  * raised. */
final object HandleExceptionType extends ChiselEnum {
  val load = Value("b00".U)
  val store = Value("b01".U)
}

/** A Chisel Bundle describing a handle exception. */
final class HandleException(implicit p: Parameters) extends BoomBundle()(p) {
  /** The bit pattern used to denote this type of handle exception.
    *
    * This is used in various places to pass the kind of handle exception back
    * to software through CSRs, because there is a single handle fault cause
    * number. */
  val xcpt_type = HandleExceptionType()

  /** The handle that was faulted on. */
  val handle = new Handle()

  /** Convert this [[HandleException]] to hardware Bits. */
  def toBits(): Bits = {
    val haddr = handle.asUInt
    // The ## and Cat methods are the same here. KGH just wanted documentation
    // on 2 ways to achieve the same thing until this gets unified behind some
    // kind of useful abstraction.
    // val result = Cat(xcpt_type.asUInt.pad(2), haddr(xLen - 2 - 1, 0))
    val result = xcpt_type.asUInt.pad(2) ## haddr.tail(2)
    require(result.getWidth == xLen, "Modified faulting haddr must be xLen wide")
    result
  }
}

/** Companion object to build [[HandleException]]s more conveniently. */
final object HandleException {
  def apply(xcpt: HandleExceptionType.Type, handle: Handle)(implicit p: Parameters): HandleException = {
    val xcpt = Wire(new HandleException())
    xcpt.xcpt_type := xcpt
    xcpt.handle := handle
    xcpt
  }
}

/** HTLB control registers. A load from a control handle reads register
 * (offset / 8). The HTLB completes the load without a memory access.
 *
 * A store to a control handle runs a command. The HTLB runs it after the
 * ROB commits the store and all older stores are done, so a command never
 * runs on a wrong path. Younger instructions can still see the old HTLB
 * state: software puts a FENCE after the store to order them.
 */
object HandleControl {
  /** Test register. A read returns MAGIC with the HID in the low 32 bits. */
  val REG_MAGIC_TEST: Int = 0
  val MAGIC: BigInt = BigInt("79756b6f00000000", 16) // "yuko" in the high word

  /** Store only. Invalidates the HTLB entry of the HID. The data is ignored. */
  val REG_INVALIDATE: Int = 1
}
