package com.team4099.lib.geometry

data class Transform2d(var m_translation: Translation2d, var m_rotation: Rotation2d) {
  constructor(
    initial: Pose2d,
    last: Pose2d
  ) : this(
    last.translation.minus(initial.translation).rotateBy(initial.rotation.unaryMinus()),
    last.rotation.minus(initial.rotation)
  )

  operator fun times(scalar: Double): Transform2d {
    return Transform2d(m_translation.times(scalar), m_rotation.times(scalar))
  }

  operator fun div(scalar: Double): Transform2d {
    return times(1.0 / scalar)
  }

  operator fun plus(other: Transform2d): Transform2d {
    return Transform2d(Pose2d(), Pose2d().transformBy(this).transformBy(other))
  }

  fun inverse(): Transform2d {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    return Transform2d(
      m_translation.unaryMinus().rotateBy(m_rotation.unaryMinus()), m_rotation.unaryMinus()
    )
  }
}
