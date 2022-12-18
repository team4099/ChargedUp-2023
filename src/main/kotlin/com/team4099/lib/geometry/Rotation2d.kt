package com.team4099.lib.geometry

import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import edu.wpi.first.math.MathUtil
import kotlin.math.abs

data class Rotation2d(var m_cos: Double, var m_sin: Double) {
  constructor() : this(1.0, 0.0)

  constructor(value: Angle) : this(value.cos, value.sin)

  constructor(
    wpilib_rotation2d: Rotation2dWPILIB
  ) : this(wpilib_rotation2d.cos, wpilib_rotation2d.sin)

  init {
    var magnitude: Double = Math.hypot(m_cos, m_sin)
    if (magnitude > 1e-6) {
      m_sin = m_sin / magnitude
      m_cos = m_cos / magnitude
    } else {
      m_sin = 0.0
      m_cos = 1.0
    }
  }

  val rotation2d: Rotation2dWPILIB = Rotation2dWPILIB(m_cos, m_sin)

  operator fun plus(other: Rotation2d): Rotation2d {
    return rotateBy(other)
  }

  operator fun minus(other: Rotation2d): Rotation2d {
    return rotateBy(other.unaryMinus())
  }

  operator fun unaryMinus(): Rotation2d {
    return Rotation2d(m_cos, -m_sin)
  }

  operator fun times(scalar: Double): Rotation2d {
    return Rotation2d(theta * scalar)
  }

  operator fun div(scalar: Double): Rotation2d {
    return times(1.0 / scalar)
  }

  fun rotateBy(other: Rotation2d): Rotation2d {
    return Rotation2d(
      m_cos * other.m_cos - m_sin * other.m_sin, m_cos * other.m_sin + m_sin * other.m_cos
    )
  }

  val theta: Angle = Math.atan2(m_sin, m_cos).radians

  val tan: Double = m_sin / m_cos

  fun interpolate(endValue: Rotation2d, t: Double): Rotation2d? {
    return plus((endValue - this) * MathUtil.clamp(t, 0.0, 1.0))
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Rotation2d) return false

    if (abs(m_cos - other.m_cos) > 1E-9) return false
    if (abs(m_sin - other.m_sin) > 1E-9) return false

    return true
  }
}
