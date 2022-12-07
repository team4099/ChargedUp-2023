package com.team4099.lib.geometry

import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N3

data class Rotation3d(val quaternion : Quaternion) {

  init {
    val m_q: Quaternion = quaternion.normalize()
  }

  constructor(roll: Angle, pitch: Angle, yaw: Angle) : this (
    Quaternion(
      Math.cos(roll.inRadians * 0.5) * Math.cos(pitch.inRadians * 0.5) * Math.cos(yaw.inRadians * 0.5) + Math.sin(roll.inRadians * 0.5) * Math.sin(pitch.inRadians * 0.5) * Math.sin(yaw.inRadians * 0.5).meters,
      Math.sin(roll.inRadians * 0.5) * Math.cos(pitch.inRadians * 0.5) * Math.cos(yaw.inRadians * 0.5) - Math.cos(roll.inRadians * 0.5) * Math.sin(pitch.inRadians * 0.5) * Math.sin(yaw.inRadians * 0.5),
      Math.cos(roll.inRadians * 0.5) * Math.sin(pitch.inRadians * 0.5) * Math.cos(yaw.inRadians * 0.5) + Math.sin(roll.inRadians * 0.5) * Math.cos(pitch.inRadians * 0.5) * Math.sin(yaw.inRadians * 0.5),
      Math.cos(roll.inRadians * 0.5) * Math.cos(pitch.inRadians * 0.5) * Math.sin(yaw.inRadians * 0.5) - Math.sin(roll.inRadians * 0.5) * Math.sin(pitch.inRadians * 0.5) * Math.cos(yaw.inRadians * 0.5)
    )
    )

  constructor(axis: Vector<N3>, angle: Angle) : this (
    Quaternion(
      Math.cos(angle.inRadians / 2.0).meters,
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(0, 0),
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(1, 0),
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(2, 0)
    )
    )


}
