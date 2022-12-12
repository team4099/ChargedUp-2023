package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

data class Pose3d(val m_translation: Translation3d, val m_rotation: Rotation3d) {
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
    Rotation3d(0.0.radians, 0.0.radians, pose.rotation.getRadians())
  )

  operator fun plus(other: Transform3d): Pose3d {
    return transformBy(other)
  }

  operator fun minus(other: Pose3d): Transform3d {
    val pose: Pose3d = relativeTo(other)
    return Transform3d(pose.getTranslation(), pose.getRotation())
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

  operator fun times(scalar: Double): Pose3d {
    return Pose3d(m_translation.times(scalar), m_rotation.times(scalar)!!)
  }

  operator fun div(scalar: Double): Pose3d {
    return times(1.0 / scalar)
  }

  fun transformBy(other: Transform3d): Pose3d {
    return Pose3d(
      m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
      other.getRotation().plus(m_rotation)
    )
  }

  fun relativeTo(other: Pose3d): Pose3d {
    val transform = Transform3d(other, this)
    return Pose3d(transform.getTranslation(), transform.getRotation())
  }

  fun exp(twist: Twist3d): Pose3d {
    val Omega: Matrix<N3, N3> =
      rotationVectorToMatrix(
        VecBuilder.fill(twist.rx.inRadians, twist.ry.inRadians, twist.rz.inRadians)
      )
    val OmegaSq: Matrix<N3, N3> = Omega.times(Omega)
    val thetaSq: Double =
      twist.rx.inRadians * twist.rx.inRadians +
        twist.ry.inRadians * twist.ry.inRadians +
        twist.rz.inRadians * twist.rz.inRadians

    // Get left Jacobian of SO3. See first line in right column of
    // http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17_identities.pdf
    val J: Matrix<N3, N3>
    J =
      if (thetaSq < 1E-9 * 1E-9) {
        // J = I + 0.5ω
        Matrix.eye(Nat.N3()).plus(Omega.times(0.5))
      } else {
        val theta = Math.sqrt(thetaSq)
        // J = I + (1 − cos(θ))/θ² ω + (θ − sin(θ))/θ³ ω²
        Matrix.eye(Nat.N3())
          .plus(Omega.times((1.0 - Math.cos(theta)) / thetaSq))
          .plus(OmegaSq.times((theta - Math.sin(theta)) / (thetaSq * theta)))
      }

    // Get translation component
    val translation: Matrix<N3, N1> =
      J.times(
        MatBuilder(Nat.N3(), Nat.N1())
          .fill(twist.dx.inMeters, twist.dy.inMeters, twist.dz.inMeters)
      )
    val transform =
      Transform3d(
        Translation3d(
          translation.get(0, 0).meters,
          translation.get(1, 0).meters,
          translation.get(2, 0).meters
        ),
        Rotation3d(twist.rx, twist.ry, twist.rz)
      )
    return this.plus(transform)
  }

  fun log(end: Pose3d): Twist3d {
    val transform = end.relativeTo(this)
    val rotVec = transform.getRotation().quaternion.toRotationVector()
    val Omega: Matrix<N3, N3> = rotationVectorToMatrix(rotVec)
    val OmegaSq: Matrix<N3, N3> = Omega.times(Omega)
    val thetaSq =
      rotVec[0, 0] * rotVec[0, 0] + rotVec[1, 0] * rotVec[1, 0] + rotVec[2, 0] * rotVec[2, 0]

    // Get left Jacobian inverse of SO3. See fourth line in right column of
    // http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17_identities.pdf
    val Jinv: Matrix<N3, N3>
    Jinv =
      if (thetaSq < 1E-9 * 1E-9) {
        // J⁻¹ = I − 0.5ω + 1/12 ω²
        Matrix.eye(Nat.N3()).minus(Omega.times(0.5)).plus(OmegaSq.times(1.0 / 12.0))
      } else {
        val theta = Math.sqrt(thetaSq)
        val halfTheta = 0.5 * theta

        // J⁻¹ = I − 0.5ω + (1 − 0.5θ cos(θ/2) / sin(θ/2))/θ² ω²
        Matrix.eye(Nat.N3())
          .minus(Omega.times(0.5))
          .plus(
            OmegaSq.times(
              (1.0 - 0.5 * theta * Math.cos(halfTheta) / Math.sin(halfTheta)) / thetaSq
            )
          )
      }

    // Get dtranslation component
    val dtranslation: Matrix<N3, N1> =
      Jinv.times(
        MatBuilder(Nat.N3(), Nat.N1())
          .fill(
            transform.getX().inMeters,
            transform.getY().inMeters,
            transform.getZ().inMeters
          )
      )
    return Twist3d(
      dtranslation.get(0, 0).meters,
      dtranslation.get(1, 0).meters,
      dtranslation.get(2, 0).meters,
      rotVec[0, 0].radians,
      rotVec[1, 0].radians,
      rotVec[2, 0].radians
    )
  }

  fun toPose2d(): Pose2d {
    return Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d())
  }

  private fun rotationVectorToMatrix(rotation: Vector<N3>): Matrix<N3, N3> {
    // Given a rotation vector <a, b, c>,
    //         [ 0 -c  b]
    // Omega = [ c  0 -a]
    //         [-b  a  0]
    return MatBuilder(Nat.N3(), Nat.N3())
      .fill(
        0.0,
        -rotation.get(2, 0),
        rotation.get(1, 0),
        rotation.get(2, 0),
        0.0,
        -rotation.get(0, 0),
        -rotation.get(1, 0),
        rotation.get(0, 0),
        0.0
      )
  }
}
