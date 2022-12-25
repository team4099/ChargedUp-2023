package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Rotation2dWPILIB
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.geometry.Translation2dWPILIB
import com.team4099.lib.geometry.Twist2dWPILIB
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
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.inRadiansPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  val gyroInputs = GyroIO.GyroIOInputs()
  val swerveModules = swerveModuleIOs.getSwerveModules()

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
    Translation2d(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val frontRightWheelLocation =
    Translation2d(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backLeftWheelLocation =
    Translation2d(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backRightWheelLocation =
    Translation2d(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )

  val swerveDriveKinematics =
    SwerveDriveKinematics(
      frontLeftWheelLocation.translation2d,
      frontRightWheelLocation.translation2d,
      backLeftWheelLocation.translation2d,
      backRightWheelLocation.translation2d
    )

  var setPointStates =
    mutableListOf(
      SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
    )

  var odometryPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, Rotation2d(0.0.radians))

  var targetPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, Rotation2d(0.0.radians))

  var lastModulePositions = mutableListOf(0.0.meters, 0.0.meters, 0.0.meters, 0.0.meters)
  var lastGyroPosition = 0.0.radians

  override fun periodic() {
    gyroIO.updateInputs(gyroInputs)
    swerveModules.forEach { it.periodic() }

    // updating odometry every loop cycle
    val measuredStatesDifference = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0 until 4) {
      measuredStatesDifference[i] =
        SwerveModuleState(
          (swerveModules[i].inputs.drivePosition - lastModulePositions[i]).inMeters,
          swerveModules[i].inputs.steeringPosition.inRotation2ds
        )
      lastModulePositions[i] = swerveModules[i].inputs.drivePosition
    }
    val chassisStateDiff: ChassisSpeeds =
      swerveDriveKinematics.toChassisSpeeds(*measuredStatesDifference)

    if (gyroInputs.gyroConnected) {
      odometryPose =
        Pose2d(
          odometryPose.pose2d.exp(
            Twist2dWPILIB(
              chassisStateDiff.vxMetersPerSecond,
              chassisStateDiff.vyMetersPerSecond,
              gyroInputs.gyroYaw.inRadians - lastGyroPosition.inRadians
            )
          )
        )
    } else {
      odometryPose =
        Pose2d(
          odometryPose.pose2d.exp(
            Twist2dWPILIB(
              chassisStateDiff.vxMetersPerSecond,
              chassisStateDiff.vyMetersPerSecond,
              chassisStateDiff.omegaRadiansPerSecond
            )
          )
        )
    }

    lastGyroPosition = gyroInputs.gyroYaw

    // Update field velocity
    val measuredStates = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0..3) {
      measuredStates[i] =
        SwerveModuleState(
          swerveModules[i].inputs.driveVelocity.inMetersPerSecond,
          swerveModules[i].inputs.steeringPosition.inRotation2ds
        )
    }
    val chassisState: ChassisSpeeds = swerveDriveKinematics.toChassisSpeeds(*measuredStates)
    val fieldVelocity =
      Translation2dWPILIB(chassisState.vxMetersPerSecond, chassisState.vyMetersPerSecond)
        .rotateBy(
          odometryPose
            .rotation
            .rotation2d
        ) // we don't use this but it's there if you want it ig

    Logger.getInstance().processInputs("Drivetrain/Gyro", gyroInputs)
    Logger.getInstance().recordOutput("Drivetrain/ModuleStates", *measuredStates)
    Logger.getInstance().recordOutput("Drivetrain/SetPointStates", *setPointStates.toTypedArray())

    Logger.getInstance()
      .recordOutput(
        "Odometry/pose",
        doubleArrayOf(
          odometryPose.x.inMeters, odometryPose.y.inMeters, odometryPose.theta.inRadians
        )
      )
    Logger.getInstance()
      .recordOutput(
        "Odometry/targetPose",
        doubleArrayOf(targetPose.x.inMeters, targetPose.y.inMeters, targetPose.theta.inRadians)
      )
  }

  /**
   * @param angularVelocity Represents the angular velocity of the chassis
   * @param driveVector Pair of linear velocities: First is X vel, second is Y vel
   * @param fieldOriented Are the chassis speeds driving relative to field (aka use gyro or not)
   */
  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    fieldOriented: Boolean = true
  ) {
    if (angularVelocity.inRadiansPerSecond.absoluteValue < 1E-3 &&
      driveVector.first.inMetersPerSecond.absoluteValue < 1E-3 &&
      driveVector.second.inMetersPerSecond.absoluteValue < 1E-3
    ) {
      for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
        swerveModules[moduleIndex].setOpenLoop(
          swerveModules[moduleIndex].inputs.steeringPosition, 0.0
        )
      }
    } else {
      val swerveModuleStates: Array<SwerveModuleState>
      if (gyroInputs.gyroConnected && fieldOriented) {
        swerveModuleStates =
          swerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              driveVector.first.inMetersPerSecond,
              driveVector.second.inMetersPerSecond,
              angularVelocity.inRadiansPerSecond,
              Rotation2dWPILIB(gyroInputs.gyroYaw.cos, gyroInputs.gyroYaw.sin)
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
      setPointStates = swerveModuleStates.toMutableList()

      for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
        swerveModules[moduleIndex].setPositionOpenLoop(swerveModuleStates[moduleIndex])
      }
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
    val accelSwerveModuleStates: Array<SwerveModuleState>?
    if (fieldOriented) {
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVector.first.inMetersPerSecond,
            driveVector.second.inMetersPerSecond,
            angularVelocity.inRadiansPerSecond,
            Rotation2dWPILIB(gyroInputs.gyroYaw.cos, gyroInputs.gyroYaw.sin)
          )
        )
      accelSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveAcceleration.first.inMetersPerSecondPerSecond,
            driveAcceleration.second.inMetersPerSecondPerSecond,
            angularAcceleration.inRadiansPerSecondPerSecond,
            Rotation2dWPILIB(gyroInputs.gyroYaw.cos, gyroInputs.gyroYaw.sin)
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

  fun resetModuleZero() {
    swerveModules.forEach { it.resetModuleZero() }
  }

  /** Zeros all the sensors on the drivetrain. */
  fun zeroSensors() {
    zeroGyroYaw(0.0.degrees)
    zeroGyroPitch(0.0.degrees)
    zeroSteering()
    zeroDrive()
  }

  /**
   * Sets the gyroOffset in such a way that when added to the gyro angle it gives back toAngle.
   *
   * @param toAngle Zeros the gyro to the value
   */
  fun zeroGyroYaw(toAngle: Angle = 0.degrees) {
    gyroIO.zeroGyroYaw(toAngle)
    odometryPose.theta = toAngle
  }

  fun zeroGyroPitch(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroPitch(toAngle)
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
