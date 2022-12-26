package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.derived.Angle

data class Pose2d(var m_translation: Translation2d, var m_rotation: Rotation2d) {
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

  var x: Length = m_translation.x

  var y: Length = m_translation.y

  val rotation: Rotation2d = m_rotation

  var theta: Angle = m_rotation.theta

  val pose2d: Pose2dWPILIB = Pose2dWPILIB(translation.translation2d, rotation.rotation2d)

  operator fun times(scalar: Double): Pose2d {
    return Pose2d(m_translation.times(scalar), m_rotation.times(scalar))
  }

  operator fun div(scalar: Double): Pose2d {
    return times(1.0 / scalar)
  }

  fun transformBy(other: Transform2d): Pose2d {
    return Pose2d(pose2d.transformBy(other.transform2d))
  }

  fun relativeTo(other: Pose2d): Pose2d {
    val transform = Transform2d(other, this)
    return Pose2d(transform.m_translation, transform.m_rotation)
  }

  fun exp(twist: Twist2d): Pose2d {
    return Pose2d(pose2d.exp(twist.twist2d))
  }

  fun log(end: Pose2d): Twist2d {
    return Twist2d(pose2d.log(end.pose2d))
  }

  fun interpolate(endValue: Pose2d, t: Time): Pose2d {
    return Pose2d(pose2d.interpolate(endValue.pose2d, t.inSeconds))
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Pose2d) return false

    if ((x - other.x).absoluteValue.value > 1E-9) return false
    if ((y - other.y).absoluteValue.value > 1E-9) return false

    if (rotation != other.rotation) return false

    if ((theta - other.theta).absoluteValue.value > 1E-9) return false

    return true
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
