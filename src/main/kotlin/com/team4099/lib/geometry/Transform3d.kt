package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length

class Transform3d(val m_translation: Translation3d, val m_rotation: Rotation3d) {
  val transform3d: Transform3dWPILIB =
    Transform3dWPILIB(m_translation.translation3d, m_rotation.rotation3d)

  constructor(
    initial: Pose3d,
    last: Pose3d
  ) : this(
    last.translation.minus(initial.translation).rotateBy(initial.rotation3d.unaryMinus()),
    last.rotation3d.minus(initial.rotation3d)
  )

  constructor() : this(Translation3d(), Rotation3d())

  operator fun times(scalar: Double): Transform3d {
    return Transform3d(m_translation.times(scalar), m_rotation.times(scalar)!!)
  }

  operator fun div(scalar: Double): Transform3d {
    return times(1.0 / scalar)
  }

  operator fun plus(other: Transform3d?): Transform3d {
    return Transform3d(Pose3d(), Pose3d().transformBy(this).transformBy(other!!))
  }

  val translation3d: Translation3d = m_translation

  val x: Length = m_translation.x

  val y: Length = m_translation.y

  val z: Length = m_translation.z

  val rotation3d: Rotation3d = m_rotation

  fun inverse(): Transform3d {
    return Transform3d(
      translation3d.unaryMinus().rotateBy(rotation3d.unaryMinus()), rotation3d.unaryMinus()
    )
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Transform3d) return false

    if (m_translation != other.m_translation) return false
    if (m_rotation != other.m_rotation) return false

    return true
  }
}
