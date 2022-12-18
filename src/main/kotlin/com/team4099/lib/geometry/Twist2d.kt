package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians

data class Twist2d(val dx: Length, val dy: Length, val dtheta: Angle) {
  val twist2d: Twist2dWPILIB = Twist2dWPILIB(dx.inMeters, dy.inMeters, dtheta.inRadians)

  constructor() : this(0.meters, 0.meters, 0.radians)

  constructor(
    twist2dWPILIB: Twist2dWPILIB
  ) : this(
    twist2dWPILIB.dx.meters,
    twist2dWPILIB.dy.meters,
    twist2dWPILIB.dtheta.radians,
  )

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Twist2d) return false

    if ((dtheta - other.dtheta).absoluteValue.value > 1E-9) return false
    if ((dx - other.dx).absoluteValue.value > 1E-9) return false
    if ((dy - other.dy).absoluteValue.value > 1E-9) return false

    return true
  }
}
