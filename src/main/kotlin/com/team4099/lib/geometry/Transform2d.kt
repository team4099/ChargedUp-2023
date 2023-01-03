package com.team4099.lib.geometry

import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import kotlin.math.abs

data class Transform2d(var m_translation: Translation2d, var m_rotation: Rotation2d) {
  val transform2d: Transform2dWPILIB =
    Transform2dWPILIB(m_translation.translation2d, m_rotation.rotation2d)

  constructor(
    initial: Pose2d,
    last: Pose2d
  ) : this(
    last.translation.minus(initial.translation).rotateBy(initial.rotation.unaryMinus()),
    last.rotation.minus(initial.rotation)
  )

  constructor(
    transform2dWPILIB: Transform2dWPILIB
  ) : this(Translation2d(transform2dWPILIB.translation), Rotation2d(transform2dWPILIB.rotation))

  operator fun times(scalar: Double): Transform2d {
    return Transform2d(m_translation * scalar, m_rotation * scalar)
  }

  operator fun div(scalar: Double): Transform2d {
    return times(1.0 / scalar)
  }

  operator fun plus(other: Transform2d): Transform2d {
    return Transform2d(transform2d.plus(other.transform2d))
  }

  fun inverse(): Transform2d {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    return Transform2d(
      m_translation.unaryMinus().rotateBy(m_rotation.unaryMinus()), m_rotation.unaryMinus()
    )
  }

  /** Logical inverse of exp. */
  fun log(): Twist2d {
    val dTheta: Double = this.m_rotation.theta.inRadians
    val halfdTheta = 0.5 * dTheta
    val cosMinusOne: Double = this.m_rotation.theta.cos - 1.0
    val halfThetaByTanOfHalfdTheta: Double =
      if (abs(cosMinusOne) < 1E-9) {
        1.0 - 1.0 / 12.0 * dTheta * dTheta
      } else {
        -(halfdTheta * this.m_rotation.theta.sin) / cosMinusOne
      }
    val translationPart: Translation2d =
      this.m_translation.rotateBy(Rotation2d(halfThetaByTanOfHalfdTheta, -halfdTheta))
    return Twist2d(translationPart.x, translationPart.y, dTheta.radians)
  }

  companion object {
    /**
     * Obtain a new Transform2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    fun exp(delta: Twist2d): Transform2d {
      val sinTheta = delta.dtheta.sin
      val cosTheta = delta.dtheta.cos
      val s: Double
      val c: Double
      if (abs(delta.dtheta.inRadians) < 1E-9) {
        s = 1.0 - 1.0 / 6.0 * delta.dtheta.inRadians * delta.dtheta.inRadians
        c = .5 * delta.dtheta.inRadians
      } else {
        s = sinTheta / delta.dtheta.inRadians
        c = (1.0 - cosTheta) / delta.dtheta.inRadians
      }

      return Transform2d(
        Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        Rotation2d(cosTheta, sinTheta)
      )
    }
  }
}
