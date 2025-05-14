//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.v3

package object common extends
  boom.v3.common.constants.ScalarOpConstants with
  boom.v3.common.constants.ExcCauseConstants with
  boom.v3.common.constants.RISCVConstants with
  boom.v3.common.constants.IQType
{
  /** Logical implication (p -> q) operator in Chisel hardware.
   * This function is mostly useful inside of asserts.
   *
   * For reference:
   *   p   |   q   | implies(p, q)
   * ------+-------+--------------
   * True  | True  | True
   * True  | False | False
   * False | True  | True
   * False | False | True
   */
  def implies(p: chisel3.Bool, q: chisel3.Bool): chisel3.Bool = !p || q
}
