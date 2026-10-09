package boom.v3.common

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

/** Class meant for debugging components with Perfetto.
 *
 * Uses Midas (Golden Gate) synthesized printf's so that FireSim simulations
 * can be played into Perfetto for visualization and debugging.
 *
 */
class TimelineTracker()(implicit val p: Parameters) extends HasBoomCoreParameters {
  val cycle = if (boomParams.enableStateTracing) {
    val c = RegInit(0.U(64.W))
    c := c + 1.U
    c
  } else { 0.U }

  def mark(thread: String, event: String): Unit = {
    if (boomParams.enableStateTracing) {
      midas.targetutils.SynthesizePrintf(printf(s"TL(M,$thread,$event,%d)\n", cycle))
    }
  }

  def trackStateValue(thread: String, stateName: String, stateValue: UInt, sig: Bool): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle  = RegInit(0.U(64.W))
      val valueAtStart = RegInit(0.U(stateValue.getWidth.W))
      val prev        = RegNext(sig, false.B)
      when(sig && !prev) {
        startCycle   := cycle
        valueAtStart := stateValue
      }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$stateName %x,%d,%d,0)\n", valueAtStart, cycle, startCycle)
        )
      }
    }
  }

  def trackState(thread: String, event: String, sig: Bool, value: UInt = 0.U): Unit = {
    if (boomParams.enableStateTracing) {
      val startCycle = RegInit(0.U(64.W))
      val prev       = RegNext(sig, false.B)
      when(sig && !prev) { startCycle := cycle }
      when(!sig && prev) {
        midas.targetutils.SynthesizePrintf(
          printf(s"TL(B,$thread,$event,%d,%d,%d)\n", cycle, startCycle, value)
        )
      }
    }
  }

  def trackCounter(thread: String, count: UInt): Unit = {
    if (boomParams.enableStateTracing) {
      val prev = RegNext(count, 0.U)
      when(count =/= prev) {
        midas.targetutils.SynthesizePrintf(printf(s"TL(C,$thread,%d,%d)\n", cycle, count))
      }
    }
  }
}
