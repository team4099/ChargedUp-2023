package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.radians

data class Twist3d(
  val dx: Length,
  val dy: Length,
  val dz: Length,
  val rx: Angle,
  val ry: Angle,
  val rz: Angle
) {
  constructor() :
    this(
      0.0.meters,
      0.0.meters,
      0.0.meters,
      0.0.radians,
      0.0.radians,
      0.0.radians,
    )
}
