package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.geometry.Pose
import com.team4099.lib.geometry.Translation
import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotation2ds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import com.team4099.lib.units.derived.times
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.inRadiansPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Drivetrain(val io: DrivetrainIO) : SubsystemBase() {
  val inputs = DrivetrainIO.DrivetrainIOInputs()

  val swerveModules = io.getSwerveModules()

  init {
    // Wheel speeds
    zeroSteering()
  }

  // FL, FR, BL, BR
  private val wheelSpeeds =
    mutableListOf(0.feet.perSecond, 0.feet.perSecond, 0.feet.perSecond, 0.feet.perSecond)

  private val wheelAngles = mutableListOf(0.radians, 0.radians, 0.radians, 0.radians)

  private val wheelAccelerations =
    mutableListOf(
      0.feet.perSecond.perSecond,
      0.feet.perSecond.perSecond,
      0.feet.perSecond.perSecond,
      0.feet.perSecond.perSecond
    )

  private val frontLeftWheelLocation =
    Translation(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val frontRightWheelLocation =
    Translation(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backLeftWheelLocation =
    Translation(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backRightWheelLocation =
    Translation(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )

  val swerveDriveKinematics =
    SwerveDriveKinematics(
      frontLeftWheelLocation.translation2d,
      frontRightWheelLocation.translation2d,
      backLeftWheelLocation.translation2d,
      backRightWheelLocation.translation2d
    )

  private val swerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      inputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.position }.toTypedArray()
    )

  var pose: Pose
    get() = Pose(swerveDriveOdometry.poseMeters)
    set(value) {
      swerveDriveOdometry.resetPosition(
        inputs.gyroYaw.inRotation2ds,
        swerveModules.map { it.position }.toTypedArray(),
        value.pose2d
      )
      zeroGyroYaw(pose.theta)
    }

  var targetPose: Pose = Pose(0.meters, 0.meters, 0.radians)

  override fun periodic() {
    io.updateInputs(inputs)
    swerveModules.forEach { it.periodic() }
    updateOdometry()

    Logger.getInstance().processInputs("Drivetrain", inputs)

    //    Logger.getInstance()
    //      .recordOutput("Drivetrain/frontLeftSpeedMetersPerSecond",
    // wheelSpeeds[0].inMetersPerSecond)
    //    Logger.getInstance()
    //      .recordOutput("Drivetrain/frontRightSpeedMetersPerSecond",
    // wheelSpeeds[1].inMetersPerSecond)
    //    Logger.getInstance()
    //      .recordOutput("Drivetrain/backLeftSpeedMetersPerSecond",
    // wheelSpeeds[2].inMetersPerSecond)
    //    Logger.getInstance()
    //      .recordOutput("Drivetrain/backRightSpeedMetersPerSecond",
    // wheelSpeeds[3].inMetersPerSecond)

    // Wheel angles
    //    Logger.getInstance().recordOutput("Drivetrain/frontLeftAngleRadians",
    // wheelAngles[0].inRadians)
    //    Logger.getInstance().recordOutput("Drivetrain/frontRightAngleRadians",
    // wheelAngles[1].inRadians)
    //    Logger.getInstance().recordOutput("Drivetrain/backLeftAngleRadians",
    // wheelAngles[2].inRadians)
    //    Logger.getInstance().recordOutput("Drivetrain/backRightAngleRadians",
    // wheelAngles[3].inRadians)

    Logger.getInstance()
      .recordOutput(
        "Odometry/pose", doubleArrayOf(pose.x.inMeters, pose.y.inMeters, pose.theta.inRadians)
      )
    Logger.getInstance()
      .recordOutput(
        "Odometry/targetPose",
        doubleArrayOf(targetPose.x.inMeters, targetPose.y.inMeters, targetPose.theta.inRadians)
      )
  }

  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    fieldOriented: Boolean = true
  ) {
    var swerveModuleStates: Array<SwerveModuleState>?
    if (fieldOriented) {
      swerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVector.first.inMetersPerSecond,
            driveVector.second.inMetersPerSecond,
            angularVelocity.inRadiansPerSecond,
            Rotation2d(inputs.gyroYaw.cos, inputs.gyroYaw.sin)
          )
        )
    } else {
      swerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds(
            driveVector.first.inMetersPerSecond,
            driveVector.second.inMetersPerSecond,
            angularVelocity.inRadiansPerSecond
          )
        )
    }

    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionOpenLoop(swerveModuleStates[moduleIndex])
    }
  }

  /**
   * Sets the drivetrain to the specified angular and X & Y velocities based on the current angular
   * and linear acceleration. Calculates both angular and linear velocities and acceleration and
   * calls setPositionClosedLoop for each SwerveModule object.
   *
   * @param angularVelocity The angular velocity of a specified drive
   * @param driveVector.first The linear velocity on the X axis
   * @param driveVector.second The linear velocity on the Y axis
   * @param angularAcceleration The angular acceleration of a specified drive
   * @param driveAcceleration.first The linear acceleration on the X axis
   * @param driveAcceleration.second The linear acceleration on the Y axis
   * @param fieldOriented Defines whether module states are calculated relative to field
   */
  fun setClosedLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    angularAcceleration: AngularAcceleration = 0.0.radians.perSecond.perSecond,
    driveAcceleration: Pair<LinearAcceleration, LinearAcceleration> =
      Pair(0.0.meters.perSecond.perSecond, 0.0.meters.perSecond.perSecond),
    fieldOriented: Boolean = true,
  ) {
    var velSwerveModuleStates: Array<SwerveModuleState>?
    var accelSwerveModuleStates: Array<SwerveModuleState>?
    if (fieldOriented) {
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVector.first.inMetersPerSecond,
            driveVector.second.inMetersPerSecond,
            angularVelocity.inRadiansPerSecond,
            Rotation2d(inputs.gyroYaw.cos, inputs.gyroYaw.sin)
          )
        )
      accelSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveAcceleration.first.inMetersPerSecondPerSecond,
            driveAcceleration.second.inMetersPerSecondPerSecond,
            angularAcceleration.inRadiansPerSecondPerSecond,
            Rotation2d(inputs.gyroYaw.cos, inputs.gyroYaw.sin)
          )
        )
    } else {
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds(
            driveVector.first.inMetersPerSecond,
            driveVector.second.inMetersPerSecond,
            angularVelocity.inRadiansPerSecond
          )
        )
      accelSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds(
            driveAcceleration.first.inMetersPerSecondPerSecond,
            driveAcceleration.second.inMetersPerSecondPerSecond,
            angularAcceleration.inRadiansPerSecondPerSecond
          )
        )
    }

    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionClosedLoop(
        velSwerveModuleStates[moduleIndex], accelSwerveModuleStates[moduleIndex]
      )
    }
  }

  private fun updateOdometry() {
    swerveDriveOdometry.update(
      inputs.gyroYaw.inRotation2ds, swerveModules.map { it.position }.toTypedArray()
    )
  }

  private fun hypot(a: LinearVelocity, b: LinearVelocity): LinearVelocity {
    return kotlin.math.hypot(a.inMetersPerSecond, b.inMetersPerSecond).meters.perSecond
  }

  private fun atan2(a: LinearVelocity, b: LinearVelocity): Angle {
    return kotlin.math.atan2(a.inMetersPerSecond, b.inMetersPerSecond).radians
  }

  fun resetModuleZero() {
    swerveModules.forEach { it.resetModuleZero() }
  }

  /** Zeros all the sensors on the drivetrain. */
  fun zeroSensors() {
    zeroGyroYaw()
    zeroPitchYaw()
    zeroSteering()
    zeroDrive()
  }

  /**
   * Sets the gyroOffset in such a way that when added to the gyro angle it gives back toAngle.
   *
   * @param toAngle Zeros the gyro to the value
   */
  fun zeroGyroYaw(toAngle: Angle = 0.degrees) {
    io.zeroGyroYaw(toAngle)
    swerveDriveOdometry.resetPosition(
      toAngle.inRotation2ds, swerveModules.map { it.position }.toTypedArray(), pose.pose2d
    )
  }

  fun zeroPitchYaw(toAngle: Angle = 0.0.degrees) {
    io.zeroGyroPitch(toAngle)
  }

  /** Zeros the steering motors for each swerve module. */
  fun zeroSteering() {
    swerveModules.forEach { it.zeroSteering() }
  }

  /** Zeros the drive motors for each swerve module. */
  private fun zeroDrive() {
    swerveModules.forEach { it.zeroDrive() }
  }
}
