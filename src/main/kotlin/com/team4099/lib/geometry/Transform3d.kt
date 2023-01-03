package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length

class Transform3d(val translation: Translation3d, val rotation: Rotation3d) {
  val transform3d: Transform3dWPILIB =
    Transform3dWPILIB(translation.translation3d, rotation.rotation3d)

  constructor(
    initial: Pose3d,
    last: Pose3d
  ) : this(
    last.translation.minus(initial.translation).rotateBy(initial.rotation.unaryMinus()),
    last.rotation.minus(initial.rotation)
  )

  constructor() : this(Translation3d(), Rotation3d())

  operator fun times(scalar: Double): Transform3d {
    return Transform3d(translation.times(scalar), rotation.times(scalar))
  }

  operator fun div(scalar: Double): Transform3d {
    return times(1.0 / scalar)
  }

  operator fun plus(other: Transform3d?): Transform3d {
    return Transform3d(Pose3d(), Pose3d().transformBy(this).transformBy(other!!))
  }

  val x: Length = translation.x

  val y: Length = translation.y

  val z: Length = translation.z

  fun inverse(): Transform3d {
    return Transform3d(
      translation.unaryMinus().rotateBy(rotation.unaryMinus()), rotation.unaryMinus()
    )
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Transform3d) return false

    if (translation != other.translation) return false
    if (rotation != other.rotation) return false

    return true
  }
}
