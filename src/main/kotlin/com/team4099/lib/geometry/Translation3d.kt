package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.radians

data class Translation3d(val m_x: Length, val m_y: Length, val m_z: Length) {
  constructor() : this(0.0.meters, 0.0.meters, 0.0.meters)

  constructor(
    distance: Length,
    angle: Rotation3d
  ) : this(
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).getX(),
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).getY(),
    Translation3d(distance, 0.0.meters, 0.0.meters).rotateBy(angle).getZ()
  )

  fun getDistance(other: Translation3d): Length {
    return Math.sqrt(
      Math.pow((other.m_x - m_x).inMeters, 2.0) +
        Math.pow((other.m_y - m_y).inMeters, 2.0) +
        Math.pow((other.m_z - m_z).inMeters, 2.0)
    )
      .meters
  }

  fun getX(): Length {
    return m_x
  }

  fun getY(): Length {
    return m_y
  }

  fun getZ(): Length {
    return m_z
  }

  fun getNorm(): Length {
    return Math.sqrt(
      (
        m_x.inMeters * m_x.inMeters +
          m_y.inMeters * m_y.inMeters +
          m_z.inMeters * m_z.inMeters
        )
    )
      .meters
  }

  fun rotateBy(other: Rotation3d): Translation3d {
    val p = Quaternion(0.0.radians, m_x.inMeters, m_y.inMeters, m_z.inMeters)
    val qprime = other.quaternion.times(p).times(other.quaternion.inverse())
    return Translation3d(qprime.getX(), qprime.getY(), qprime.getZ())
  }

  fun toTranslation2d(): Translation2d {
    return Translation2d(m_x, m_y)
  }

  operator fun plus(other: Translation3d): Translation3d {
    return Translation3d(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z)
  }

  operator fun minus(other: Translation3d): Translation3d {
    return Translation3d(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z)
  }

  operator fun unaryMinus(): Translation3d {
    return Translation3d(-m_x, -m_y, -m_z)
  }

  operator fun times(scalar: Double): Translation3d {
    return Translation3d(m_x * scalar, m_y * scalar, m_z * scalar)
  }

  operator fun div(scalar: Double): Translation3d {
    return Translation3d(m_x / scalar, m_y / scalar, m_z / scalar)
  }
}
