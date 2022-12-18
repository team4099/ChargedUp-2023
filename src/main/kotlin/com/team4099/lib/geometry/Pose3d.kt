package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.radians

data class Pose3d(val m_translation: Translation3d, val m_rotation: Rotation3d) {
  val translation: Translation3d = m_translation

  val x: Length = m_translation.x

  val y: Length = m_translation.y

  val z: Length = m_translation.z

  val rotation3d: Rotation3d = m_rotation

  val pose3d: Pose3dWPILIB = Pose3dWPILIB(x.inMeters, y.inMeters, z.inMeters, rotation3d.rotation3d)

  constructor() : this(Translation3d(), Rotation3d())

  constructor(
    x: Length,
    y: Length,
    z: Length,
    rotation: Rotation3d
  ) : this(Translation3d(x, y, z), rotation)

  constructor(
    pose: Pose2d
  ) : this(
    Translation3d(pose.x, pose.y, 0.0.meters),
    Rotation3d(0.0.radians, 0.0.radians, pose.rotation.theta)
  )

  constructor(
    pose3dWPILIB: Pose3dWPILIB
  ) : this(Translation3d(pose3dWPILIB.translation), Rotation3d(pose3dWPILIB.rotation))

  operator fun plus(other: Transform3d): Pose3d {
    return transformBy(other)
  }

  operator fun minus(other: Pose3d): Transform3d {
    val pose: Pose3d = relativeTo(other)
    return Transform3d(pose.translation, pose.rotation3d)
  }

  operator fun times(scalar: Double): Pose3d {
    return Pose3d(m_translation * scalar, m_rotation * scalar)
  }

  operator fun div(scalar: Double): Pose3d {
    return times(1.0 / scalar)
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Pose3d) return false

    if ((x - other.x).absoluteValue.value > 1E-9) return false
    if ((y - other.y).absoluteValue.value > 1E-9) return false
    if ((z - other.z).absoluteValue.value > 1E-9) return false

    if (rotation3d != other.rotation3d) return false

    return true
  }

  fun transformBy(other: Transform3d): Pose3d {
    return Pose3d(
      m_translation.plus(other.translation3d.rotateBy(m_rotation)),
      other.rotation3d.plus(m_rotation)
    )
  }

  fun relativeTo(other: Pose3d): Pose3d {
    val transform = Transform3d(other, this)
    return Pose3d(transform.translation3d, transform.rotation3d)
  }

  fun exp(twist: Twist3d): Pose3d {
    return Pose3d(pose3d.exp(twist.twist3d))
  }

  fun log(end: Pose3d): Twist3d {
    return Twist3d(pose3d.log(end.pose3d))
  }

  fun toPose2d(): Pose2d {
    return Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d())
  }
}
