package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians

data class Twist3d(
  val dx: Length,
  val dy: Length,
  val dz: Length,
  val rx: Angle,
  val ry: Angle,
  val rz: Angle
) {
  val twist3d: Twist3dWPILIB =
    Twist3dWPILIB(dx.inMeters, dy.inMeters, dz.inMeters, rx.inRadians, ry.inRadians, rz.inRadians)

  constructor() :
    this(
      0.0.meters,
      0.0.meters,
      0.0.meters,
      0.0.radians,
      0.0.radians,
      0.0.radians,
    )

  constructor(
    twist3dWPILIB: Twist3dWPILIB
  ) : this(
    twist3dWPILIB.dx.meters,
    twist3dWPILIB.dy.meters,
    twist3dWPILIB.dz.meters,
    twist3dWPILIB.rx.radians,
    twist3dWPILIB.ry.radians,
    twist3dWPILIB.rz.radians
  )

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Twist3d) return false

    if ((dx - other.dx).absoluteValue.value > 1E-9) return false
    if ((dy - other.dy).absoluteValue.value > 1E-9) return false
    if ((dz - other.dz).absoluteValue.value > 1E-9) return false

    if ((rx - other.rx).absoluteValue.value > 1E-9) return false
    if ((ry - other.ry).absoluteValue.value > 1E-9) return false
    if ((rz - other.rz).absoluteValue.value > 1E-9) return false

    return true
  }
}
