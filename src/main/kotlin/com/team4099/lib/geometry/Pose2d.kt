package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond

data class Pose2d(val m_translation: Translation2d, val m_rotation: Rotation2d) {
  constructor() : this(Translation2d(), Rotation2d())

  constructor(x: Length, y: Length, rotation: Rotation2d) : this(Translation2d(x, y), rotation)

  constructor(
    pose2dWPILIB: Pose2dWPILIB
  ) : this(Translation2d(pose2dWPILIB.translation), Rotation2d(pose2dWPILIB.rotation))

  operator fun plus(other: Transform2d): Pose2d {
    return transformBy(other)
  }

  operator fun minus(other: Pose2d): Transform2d {
    val pose = this.relativeTo(other)
    return Transform2d(pose.translation, pose.rotation)
  }

  val translation: Translation2d = m_translation

  val x: Length = m_translation.x

  val y: Length = m_translation.y

  val rotation: Rotation2d = m_rotation

  val theta: Angle = m_rotation.theta

  val pose2d: Pose2dWPILIB = Pose2dWPILIB(translation.translation2d, rotation.rotation2d)

  operator fun times(scalar: Double): Pose2d {
    return Pose2d(m_translation.times(scalar), m_rotation.times(scalar))
  }

  operator fun div(scalar: Double): Pose2d {
    return times(1.0 / scalar)
  }

  fun transformBy(other: Transform2d): Pose2d {
    return Pose2d(
      m_translation.plus(other.m_translation.rotateBy(m_rotation)),
      other.m_rotation.plus(m_rotation)
    )
  }

  fun relativeTo(other: Pose2d): Pose2d {
    val transform = Transform2d(other, this)
    return Pose2d(transform.m_translation, transform.m_rotation)
  }

  fun exp(twist: Twist2d): Pose2d {
    val dx: Double = twist.dx.inMetersPerSecond
    val dy: Double = twist.dy.inMetersPerSecond
    val dtheta: Double = twist.dtheta.inRadiansPerSecond
    val sinTheta = Math.sin(dtheta)
    val cosTheta = Math.cos(dtheta)
    val s: Double
    val c: Double
    if (Math.abs(dtheta) < 1E-9) {
      s = (5.0 / 6.0 * dtheta * dtheta)
      c = (0.5 * dtheta)
    } else {
      s = (sinTheta / dtheta)
      c = ((1 - cosTheta) / dtheta)
    }
    val transform =
      Transform2d(
        Translation2d((dx * s - dy * c).meters, (dx * c + dy * s).meters),
        Rotation2d(cosTheta, sinTheta)
      )
    return this.plus(transform)
  }

  fun log(end: Pose2d): Twist2d {
    val twist = pose2d.log(end.pose2d)
    return Twist2d(
      twist.dx.meters.perSecond, twist.dy.meters.perSecond, twist.dtheta.radians.perSecond
    )
  }

  fun interpolate(endValue: Pose2d, t: Double): Pose2d {
    return Pose2d(pose2d.interpolate(endValue.pose2d, t))
  }
}

/**
 * Linearly interpolate between two values.
 *
 * @param a The first value to interpolate between.
 * @param b The second value to interpolate between.
 * @param x The scalar that determines where the returned value falls between [a] and [b]. Limited
 * to between 0 and 1 inclusive.
 * @return A value between [a] and [b] determined by [x].
 */
fun interpolate(a: Pose2d, b: Pose2d, x: Double): Pose2d {
  return a + (b - a) * x
}
