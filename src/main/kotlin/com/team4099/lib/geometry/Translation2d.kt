package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.sin
import kotlin.math.hypot

/**
 * Represents a translation in 3d space. This object can be used to represent a point or a vector.
 *
 * <p>This assumes that you are using conventional mathematical axes. When the robot is placed on
 * the origin, facing toward the X direction, moving forward increases the X, moving to the left
 * increases the Y, and moving upwards increases the Z.
 */
data class Translation2d(var x: Length, var y: Length) {
  constructor(translation: Translation2dWPILIB) : this(translation.x.meters, translation.y.meters)

  constructor() : this(0.0.meters, 0.0.meters)

  val translation2d
    get() = Translation2dWPILIB(x.inMeters, y.inMeters)

  val magnitude
    get() = hypot(x.inMeters, y.inMeters)

  operator fun plus(other: Translation2d): Translation2d {
    return Translation2d(x + other.x, y + other.y)
  }

  operator fun minus(other: Translation2d): Translation2d {
    return Translation2d(x - other.x, y - other.y)
  }

  operator fun times(scalar: Double): Translation2d {
    return Translation2d(x * scalar, y * scalar)
  }

  operator fun div(scalar: Double): Translation2d {
    return Translation2d(x / scalar, y / scalar)
  }

  operator fun unaryMinus(): Translation2d {
    return Translation2d(x * -1, y * -1)
  }

  fun rotateBy(other: Angle): Translation2d {
    return Translation2d(x * other.cos - y * other.sin, x * other.sin + y * other.cos)
  }

  fun normalize(): Translation2d {
    return this / magnitude
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Translation2d) return false

    if ((x - other.x).absoluteValue.value > 1E-9) return false
    if ((y - other.y).absoluteValue.value > 1E-9) return false

    return true
  }
}
