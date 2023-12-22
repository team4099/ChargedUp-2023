package com.team4099.lib.trajectory

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import org.littletonrobotics.junction.Logger

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
class CustomHolonomicDriveController(
  private val m_xController: PIDController,
  private val m_yController: PIDController,
  private val m_thetaController: PIDController
) {
  private var m_poseError = Pose2d()
  private var m_rotationError = Rotation2d()
  private var m_poseTolerance = Pose2d()
  private var m_enabled = true

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A PID controller to respond to error in angle.
   */
  init {
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI)
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  fun atReference(): Boolean {
    val eTranslate = m_poseError.translation
    val eRotate = m_rotationError
    val tolTranslate = m_poseTolerance.translation
    val tolRotate = m_poseTolerance.rotation
    return Math.abs(eTranslate.x) < tolTranslate.x &&
      Math.abs(eTranslate.y) < tolTranslate.y &&
      Math.abs(eRotate.radians) < tolRotate.radians
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  fun setTolerance(tolerance: Pose2d) {
    m_poseTolerance = tolerance
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The linear velocity reference.
   * @param angleRef The angular reference.
   * @param angleVelocityRefRadians The angular velocity reference.
   * @return The next output of the holonomic drive controller.
   */
  fun calculate(
    currentPose: Pose2d,
    poseRef: Pose2d,
    linearVelocityRefMeters: Double,
    angleRef: Rotation2d,
    angleVelocityRefRadians: Double
  ): ChassisSpeeds {

    // Calculate feedforward velocities (field-relative).
    val xFF = linearVelocityRefMeters * poseRef.rotation.cos
    val yFF = linearVelocityRefMeters * poseRef.rotation.sin
    m_poseError = poseRef.relativeTo(currentPose)
    m_rotationError = angleRef.minus(currentPose.rotation)
    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF, yFF, angleVelocityRefRadians, currentPose.rotation
      )
    }

    // Calculate feedback velocities (based on position error).
    val xFeedback = m_xController.calculate(currentPose.x, poseRef.x)
    val yFeedback = m_yController.calculate(currentPose.y, poseRef.y)
    val thetaFeedback = m_thetaController.calculate(currentPose.rotation.radians, angleRef.radians)
    Logger.getInstance().recordOutput("Pathfollow/thetaFeedbackRadians", thetaFeedback)
    Logger.getInstance().recordOutput("Pathfollow/appliedThetaFeedback", angleVelocityRefRadians + thetaFeedback)

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
      xFF + xFeedback,
      yFF + yFeedback,
      angleVelocityRefRadians + thetaFeedback,
      currentPose.rotation
    )
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param driveState The desired drive trajectory state.
   * @param holonomicRotationState The desired holonomic rotation state.
   * @return The next output of the holonomic drive controller.
   */
  fun calculate(
    currentPose: Pose2d,
    driveState: Trajectory.State,
    holonomicRotationState: RotationSequence.State
  ): ChassisSpeeds {
    return calculate(
      currentPose,
      driveState.poseMeters,
      driveState.velocityMetersPerSecond,
      holonomicRotationState.position,
      holonomicRotationState.velocityRadiansPerSec
    )
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  fun setEnabled(enabled: Boolean) {
    m_enabled = enabled
  }
}
