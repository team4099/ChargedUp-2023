package com.team4099.lib.geometry

import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N3
import kotlin.math.abs

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

  val x: Angle
    get() {
      val w: Double = m_q.w.inRadians
      val x: Double = m_q.x.inMeters
      val y: Double = m_q.y.inMeters
      val z: Double = m_q.z.inMeters

      // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
      return Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)).radians
    }

  val y: Angle
    get() {
      val w: Double = m_q.w.inRadians
      val x: Double = m_q.x.inMeters
      val y: Double = m_q.y.inMeters
      val z: Double = m_q.z.inMeters

      // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
      val ratio = 2.0 * (w * y - z * x)
      return if (Math.abs(ratio) >= 1.0) {
        Math.copySign(Math.PI / 2.0, ratio).radians
      } else {
        Math.asin(ratio).radians
      }
    }

  val z: Angle
    get() {
      val w: Double = m_q.w.inRadians
      val x: Double = m_q.x.inMeters
      val y: Double = m_q.y.inMeters
      val z: Double = m_q.z.inMeters

      // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
      return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)).radians
    }

  val axis: Vector<N3>
    get() {
      val norm =
        Math.sqrt(
          m_q.x.inMeters * m_q.x.inMeters +
            m_q.y.inMeters * m_q.y.inMeters +
            m_q.z.inMeters * m_q.z.inMeters
        )
      return if (norm == 0.0) {
        VecBuilder.fill(0.0, 0.0, 0.0)
      } else {
        VecBuilder.fill(m_q.x.inMeters / norm, m_q.y.inMeters / norm, m_q.z.inMeters / norm)
      }
    }

  val theta: Angle
    get() {
      val norm =
        Math.sqrt(
          m_q.x.inMeters * m_q.x.inMeters +
            m_q.y.inMeters * m_q.y.inMeters +
            m_q.z.inMeters * m_q.z.inMeters
        )
      return (2.0 * Math.atan2(norm, m_q.w.inRadians)).radians
    }

  val rotation3d: Rotation3dWPILIB = Rotation3dWPILIB(m_q.quaternion)

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
