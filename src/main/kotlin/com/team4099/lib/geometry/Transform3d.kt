package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length

class Transform3d(val m_translation: Translation3d, val m_rotation: Rotation3d) {

  constructor(
    initial: Pose3d,
    last: Pose3d
  ) : this(
    last.getTranslation()
      .minus(initial.getTranslation())
      .rotateBy(initial.getRotation().unaryMinus()),
    last.getRotation().minus(initial.getRotation())
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

  fun getTranslation(): Translation3d {
    return m_translation
  }

  fun getX(): Length {
    return m_translation.getX()
  }

  fun getY(): Length {
    return m_translation.getY()
  }

  fun getZ(): Length {
    return m_translation.getZ()
  }

  fun getRotation(): Rotation3d {
    return m_rotation
  }

  fun inverse(): Transform3d {
    return Transform3d(
      getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
      getRotation().unaryMinus()
    )
  }
}
