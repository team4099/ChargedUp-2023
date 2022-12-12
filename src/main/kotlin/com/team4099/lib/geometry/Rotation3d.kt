package com.team4099.lib.geometry

import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N3
import org.ejml.dense.row.factory.DecompositionFactory_DDRM
import kotlin.math.acos

data class Rotation3d(val quaternion: Quaternion) {
  val m_q: Quaternion = quaternion.normalize()

  companion object {
    fun setupRotation3d1(rotationMatrix: Matrix<N3, N3>): Quaternion {
      var R: Matrix<N3, N3> = rotationMatrix

      val trace: Double = R.get(0, 0) + R.get(1, 1) + R.get(2, 2)
      val w: Double
      val x: Double
      val y: Double
      val z: Double

      if (trace > 0.0) {
        val s = 0.5 / Math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R.get(2, 1) - R.get(1, 2)) * s
        y = (R.get(0, 2) - R.get(2, 0)) * s
        z = (R.get(1, 0) - R.get(0, 1)) * s
      } else {
        if (R.get(0, 0) > R.get(1, 1) && R.get(0, 0) > R.get(2, 2)) {
          val s = 2.0 * Math.sqrt(1.0 + R.get(0, 0) - R.get(1, 1) - R.get(2, 2))
          w = (R.get(2, 1) - R.get(1, 2)) / s
          x = 0.25 * s
          y = (R.get(0, 1) + R.get(1, 0)) / s
          z = (R.get(0, 2) + R.get(2, 0)) / s
        } else if (R.get(1, 1) > R.get(2, 2)) {
          val s = 2.0 * Math.sqrt(1.0 + R.get(1, 1) - R.get(0, 0) - R.get(2, 2))
          w = (R.get(0, 2) - R.get(2, 0)) / s
          x = (R.get(0, 1) + R.get(1, 0)) / s
          y = 0.25 * s
          z = (R.get(1, 2) + R.get(2, 1)) / s
        } else {
          val s = 2.0 * Math.sqrt(1.0 + R.get(2, 2) - R.get(0, 0) - R.get(1, 1))
          w = (R.get(1, 0) - R.get(0, 1)) / s
          x = (R.get(0, 2) + R.get(2, 0)) / s
          y = (R.get(1, 2) + R.get(2, 1)) / s
          z = 0.25 * s
        }
      }

      return Quaternion(w.radians, x, y, z)
    }

    fun setupRotation3d2(initial: Vector<N3>, last: Vector<N3>): Quaternion {
      val dot: Double = initial.dot(last)
      val normProduct: Double = initial.norm() * last.norm()
      val dotNorm = dot / normProduct

      if (dotNorm > 1.0 - 1E-9) {
        // If the dot product is 1, the two vectors point in the same direction so
        // there's no rotation. The default initialization of m_q will work.
        return Quaternion()
      } else if (dotNorm < -1.0 + 1E-9) {
        // If the dot product is -1, the two vectors point in opposite directions
        // so a 180 degree rotation is required. Any orthogonal vector can be used
        // for it. Q in the QR decomposition is an orthonormal basis, so it
        // contains orthogonal unit vectors.
        val X =
          MatBuilder(Nat.N3(), Nat.N1())
            .fill(initial.get(0, 0), initial.get(1, 0), initial.get(2, 0))
        val qr = DecompositionFactory_DDRM.qr(3, 1)
        qr.decompose(X.storage.getMatrix())
        val Q = qr.getQ(null, false)

        // w = cos(θ/2) = cos(90°) = 0
        //
        // For x, y, and z, we use the second column of Q because the first is
        // parallel instead of orthogonal. The third column would also work.
        return Quaternion(0.0.radians, Q[0, 1], Q[1, 1], Q[2, 1])
      } else {
        // initial x last
        val axis: Vector<N3> =
          VecBuilder.fill(
            initial.get(1, 0) * last[2, 0] - last[1, 0] * initial.get(2, 0),
            last[0, 0] * initial.get(2, 0) - initial.get(0, 0) * last[2, 0],
            initial.get(0, 0) * last[1, 0] - last[0, 0] * initial.get(1, 0)
          )

        // https://stackoverflow.com/a/11741520
        return Quaternion(
          (normProduct + dot).radians, axis.get(0, 0), axis.get(1, 0), axis.get(2, 0)
        )
          .normalize()
      }
    }
  }

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

  constructor(rotationMatrix: Matrix<N3, N3>) : this(setupRotation3d1(rotationMatrix))

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

  operator fun times(scalar: Double): Rotation3d? {
    // https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp
    return if (m_q.w.inRadians >= 0.0) {
      Rotation3d(
        VecBuilder.fill(m_q.x.inMeters, m_q.y.inMeters, m_q.z.inMeters),
        (2.0 * scalar * acos(m_q.w.inRadians)).radians
      )
    } else {
      Rotation3d(
        VecBuilder.fill(-m_q.x.inMeters, -m_q.y.inMeters, -m_q.z.inMeters),
        ((2.0 * scalar) * acos(-m_q.w.inRadians)).radians
      )
    }
  }
}
