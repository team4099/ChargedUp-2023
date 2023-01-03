package com.team4099.lib.geometry

import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.angle
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotation2ds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import kotlin.math.abs

data class Transform2d(val translation: Translation2d, val rotation: Angle) {
  val transform2d: Transform2dWPILIB =
    Transform2dWPILIB(translation.translation2d, rotation.inRotation2ds)

  constructor(
    initial: Pose2d,
    last: Pose2d
  ) : this(
    last.translation.minus(initial.translation).rotateBy(-(initial.rotation)),
    last.rotation.minus(initial.rotation)
  )

  constructor(
    transform2dWPILIB: Transform2dWPILIB
  ) : this(Translation2d(transform2dWPILIB.translation), transform2dWPILIB.rotation.angle)

  operator fun times(scalar: Double): Transform2d {
    return Transform2d(translation * scalar, rotation * scalar)
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

    return Transform2d(-translation.rotateBy(rotation.unaryMinus()), -rotation)
  }

  /** Logical inverse of exp. */
  fun log(): Twist2d {
    val dTheta: Double = this.rotation.inRadians
    val halfdTheta = 0.5 * dTheta
    val cosMinusOne: Double = this.rotation.cos - 1.0
    val halfThetaByTanOfHalfdTheta: Double =
      if (abs(cosMinusOne) < 1E-9) {
        1.0 - 1.0 / 12.0 * dTheta * dTheta
      } else {
        -(halfdTheta * this.rotation.sin) / cosMinusOne
      }
    val translationPart: Translation2d =
      this.translation.rotateBy(Angle(halfThetaByTanOfHalfdTheta, -halfdTheta))
    return Twist2d(translationPart.x, translationPart.y, dTheta.radians)
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (javaClass != other?.javaClass) return false

    other as Transform2d

    if (translation != other.translation) return false
    if ((rotation - other.rotation).absoluteValue.value > 1E-9) return false

    return true
  }

  override fun hashCode(): Int {
    var result = translation.hashCode()
    result = 31 * result + rotation.hashCode()
    return result
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
        Angle(cosTheta, sinTheta)
      )
    }
  }
}
