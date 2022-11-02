package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.hypot

/**
 * Represents a translation in 3d space. This object can be used to represent a point or a vector.
 *
 * <p>This assumes that you are using conventional mathematical axes. When the robot is placed on
 * the origin, facing toward the X direction, moving forward increases the X, moving to the left
 * increases the Y, and moving upwards increases the Z.
 */
data class Translation(val x: Length, val y: Length) {
  constructor(translation: Translation2d) : this(translation.x.meters, translation.y.meters)

  val translation2d
    get() = Translation2d(x.inMeters, y.inMeters)

  val magnitude
    get() = hypot(x.inMeters, y.inMeters)

  operator fun plus(other: Translation): Translation {
    return Translation(x + other.x, y + other.y)
  }

  operator fun minus(other: Translation): Translation {
    return Translation(x - other.x, y - other.y)
  }

  operator fun times(scalar: Double): Translation {
    return Translation(x * scalar, y * scalar)
  }

  operator fun div(scalar: Double): Translation {
    return Translation(x / scalar, y / scalar)
  }

  operator fun unaryMinus(): Translation {
    return Translation(x * -1, y * -1)
  }

  fun normalize(): Translation {
    return this / magnitude
  }
}
