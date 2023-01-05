package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.radians
import kotlin.math.pow
import kotlin.math.sqrt

data class Translation3d(val x: Length, val y: Length, val z: Length) {

  val translation3d: Translation3dWPILIB = Translation3dWPILIB(x.inMeters, y.inMeters, z.inMeters)

  constructor() : this(0.0.meters, 0.0.meters, 0.0.meters)

  constructor(
    distance: Length,
    angle: Rotation3d
  ) : this(
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).x,
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).y,
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).z
  )

  constructor(
    translation3dWPILIB: Translation3dWPILIB
  ) : this(translation3dWPILIB.x.meters, translation3dWPILIB.y.meters, translation3dWPILIB.z.meters)

  fun getDistance(other: Translation3d): Length {
    return sqrt(
      (other.x - x).inMeters.pow(2.0) +
        (other.y - y).inMeters.pow(2.0) +
        (other.z - z).inMeters.pow(2.0)
    )
      .meters
  }

  val norm: Length
    get() {
      return sqrt((x.inMeters * x.inMeters + y.inMeters * y.inMeters + z.inMeters * z.inMeters))
        .meters
    }

  fun rotateBy(other: Rotation3d): Translation3d {
    val p = Quaternion(0.0.radians, x.inMeters, y.inMeters, z.inMeters)
    val qprime = other.quaternion * p * other.quaternion.inverse()
    return Translation3d(qprime.x, qprime.y, qprime.z)
  }

  fun toTranslation2d(): Translation2d {
    return Translation2d(x, y)
  }

  operator fun plus(other: Translation3d): Translation3d {
    return Translation3d(x + other.x, y + other.y, z + other.z)
  }

  operator fun minus(other: Translation3d): Translation3d {
    return Translation3d(x - other.x, y - other.y, z - other.z)
  }

  operator fun unaryMinus(): Translation3d {
    return Translation3d(-x, -y, -z)
  }

  operator fun times(scalar: Double): Translation3d {
    return Translation3d(x * scalar, y * scalar, z * scalar)
  }

  operator fun div(scalar: Double): Translation3d {
    return Translation3d(x / scalar, y / scalar, z / scalar)
  }
}
