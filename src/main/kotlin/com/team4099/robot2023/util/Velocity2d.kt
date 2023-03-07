package com.team4099.robot2023.util

import edu.wpi.first.math.geometry.Translation2d
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.hypot

data class Velocity2d(val x: LinearVelocity, val y: LinearVelocity) {

  constructor() : this(0.0.meters.perSecond, 0.0.meters.perSecond)

  val velocity2dWPIlib = Translation2d(x.inMetersPerSecond, y.inMetersPerSecond)

  val magnitude = hypot(x.inMetersPerSecond, y.inMetersPerSecond).meters.perSecond

  val heading: Angle = velocity2dWPIlib.angle.angle

  companion object {
    fun fromVelocityVectorToVelocity2d(speed: LinearVelocity, heading: Angle): Velocity2d {
      return Velocity2d(speed * heading.cos, speed * heading.sin)
    }
  }

  operator fun plus(other: Velocity2d): Velocity2d {
    return Velocity2d(x + other.x, y + other.y)
  }

  operator fun minus(other: Velocity2d): Velocity2d {
    return Velocity2d(x - other.x, y - other.y)
  }

  operator fun times(scalar: Double): Velocity2d {
    return Velocity2d(x * scalar, y * scalar)
  }

  operator fun div(scalar: Double): Velocity2d {
    return Velocity2d(x / scalar, y / scalar)
  }

  operator fun unaryMinus(): Velocity2d {
    return Velocity2d(x * -1, y * -1)
  }

  fun rotateBy(other: Angle): Velocity2d {
    return Velocity2d(x * other.cos - y * other.sin, x * other.sin + y * other.cos)
  }

  fun normalize(): Velocity2d {
    return this / magnitude.inMetersPerSecond
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Velocity2d) return false

    if ((x - other.x).absoluteValue.value > 1E-9) return false
    if ((y - other.y).absoluteValue.value > 1E-9) return false

    return true
  }
}
