package com.team4099.lib.geometry

import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N3

data class Rotation3d(val quaternion: Quaternion) {
  val m_q: Quaternion = quaternion.normalize()

  constructor(rotation3dWPILIB: Rotation3dWPILIB) : this(Quaternion(rotation3dWPILIB.quaternion))

  constructor(
    roll: Angle,
    pitch: Angle,
    yaw: Angle
  ) : this(
    Quaternion(
      (
        Math.cos(roll.inRadians * 0.5) *
          Math.cos(pitch.inRadians * 0.5) *
          Math.cos(yaw.inRadians * 0.5) +
          Math.sin(roll.inRadians * 0.5) *
          Math.sin(pitch.inRadians * 0.5) *
          Math.sin(yaw.inRadians * 0.5)
        )
        .radians,
      Math.sin(roll.inRadians * 0.5) *
        Math.cos(pitch.inRadians * 0.5) *
        Math.cos(yaw.inRadians * 0.5) -
        Math.cos(roll.inRadians * 0.5) *
        Math.sin(pitch.inRadians * 0.5) *
        Math.sin(yaw.inRadians * 0.5),
      Math.cos(roll.inRadians * 0.5) *
        Math.sin(pitch.inRadians * 0.5) *
        Math.cos(yaw.inRadians * 0.5) +
        Math.sin(roll.inRadians * 0.5) *
        Math.cos(pitch.inRadians * 0.5) *
        Math.sin(yaw.inRadians * 0.5),
      Math.cos(roll.inRadians * 0.5) *
        Math.cos(pitch.inRadians * 0.5) *
        Math.sin(yaw.inRadians * 0.5) -
        Math.sin(roll.inRadians * 0.5) *
        Math.sin(pitch.inRadians * 0.5) *
        Math.cos(yaw.inRadians * 0.5)
    )
  )

  constructor(
    axis: Vector<N3>,
    angle: Angle
  ) : this(
    Quaternion(
      Math.cos(angle.inRadians / 2.0).radians,
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(0, 0),
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(1, 0),
      axis.times(1.0 / axis.norm()).times(Math.sin(angle.inRadians / 2.0)).get(2, 0)
    )
  )

  constructor(rotationMatrix: Matrix<N3, N3>) : this(Rotation3dWPILIB(rotationMatrix))

  constructor(initial: Vector<N3>, last: Vector<N3>) : this(Rotation3dWPILIB(initial, last))

  constructor() : this(Quaternion())

  fun rotateBy(other: Rotation3d): Rotation3d {
    return Rotation3d(other.m_q.times(m_q))
  }
  val rotation3d: Rotation3dWPILIB = Rotation3dWPILIB(m_q.quaternion)

  val x: Angle = rotation3d.x.radians

  val y: Angle = rotation3d.y.radians

  val z: Angle = rotation3d.z.radians

  val theta: Angle = rotation3d.angle.radians

  fun toRotation2d(): Rotation2d {
    return Rotation2d(z)
  }

  operator fun plus(other: Rotation3d): Rotation3d {
    return rotateBy(other)
  }

  operator fun minus(other: Rotation3d): Rotation3d {
    return rotateBy(other.unaryMinus())
  }

  fun unaryMinus(): Rotation3d {
    return Rotation3d(m_q.inverse())
  }

  operator fun times(scalar: Double): Rotation3d {
    return Rotation3d(rotation3d * scalar)
  }

  operator fun div(scalar: Double): Rotation3d {
    return times(1.0 / scalar)
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Rotation3d) return false

    if (m_q != other.m_q) return false

    return true
  }
}
