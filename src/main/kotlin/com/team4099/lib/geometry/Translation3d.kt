package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.meters

data class Translation3d (val m_x: Length, val m_y: Length, val m_z: Length) {
  constructor() : this (
    0.0.meters,
    0.0.meters,
    0.0.meters
    )

  constructor(distance: Length, angle: Rotation3d) : this (
    Translation3d(distance, 0.0, 0.0).rotateBy(angle)
    )
}
