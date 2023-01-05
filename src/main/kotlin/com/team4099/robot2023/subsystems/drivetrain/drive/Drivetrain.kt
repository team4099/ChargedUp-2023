package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Transform2d
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.geometry.Twist2d
import com.team4099.lib.kinematics.ChassisAccels
import com.team4099.lib.kinematics.ChassisSpeeds
import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotation2ds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.util.Alert
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  val gyroNotConnectedAlert =
    Alert(
      "Gyro is not connected, field relative driving will be significantly worse.",
      Alert.AlertType.ERROR
    )

  val gyroInputs = GyroIO.GyroIOInputs()
  val swerveModules = swerveModuleIOs.getSwerveModules()
  var gyroYawOffset = 0.0.radians

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

  var swerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray(),
    )

  var setPointStates =
    mutableListOf(
      SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
    )

  var odometryPose: Pose2d
    get() = Pose2d(swerveDriveOdometry.poseMeters)
    set(value) {
      swerveDriveOdometry.resetPosition(
        gyroInputs.gyroYaw.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        value.pose2d
      )

      if (RobotBase.isReal()) {
        zeroGyroYaw(odometryPose.rotation)
      } else {
        undriftedPose = odometryPose
      }
    }

  var undriftedPose: Pose2d = Pose2d()

  var targetPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, 0.0.radians)

  var drift: Transform2d = Transform2d(Translation2d(), 0.0.radians)

  var lastModulePositions = mutableListOf(0.0.meters, 0.0.meters, 0.0.meters, 0.0.meters)

  override fun periodic() {
    gyroNotConnectedAlert.set(!gyroInputs.gyroConnected)
    gyroIO.updateInputs(gyroInputs)

    swerveModules.forEach { it.periodic() }

    // updating odometry every loop cycle
    updateOdometry()

    // Update field velocity
    val measuredStates = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0..3) {
      measuredStates[i] =
        SwerveModuleState(
          swerveModules[i].inputs.driveVelocity.inMetersPerSecond,
          swerveModules[i].inputs.steeringPosition.inRotation2ds
        )
    }
    val chassisState: ChassisSpeeds =
      ChassisSpeeds(swerveDriveKinematics.toChassisSpeeds(*measuredStates))
    val fieldVelocity =
      Translation2d(
        chassisState.vx.inMetersPerSecond.meters, chassisState.vy.inMetersPerSecond.meters
      )
        .rotateBy(odometryPose.rotation) // we don't use this but it's there if you want it ig

    Logger.getInstance()
      .recordOutput("Drivetrain/xVelocityMetersPerSecond", fieldVelocity.x.inMeters)
    Logger.getInstance()
      .recordOutput("Drivetrain/yVelocityMetersPerSecond", fieldVelocity.y.inMeters)

    Logger.getInstance().processInputs("Drivetrain/Gyro", gyroInputs)
    Logger.getInstance().recordOutput("Drivetrain/ModuleStates", *measuredStates)
    Logger.getInstance().recordOutput("Drivetrain/SetPointStates", *setPointStates.toTypedArray())

    Logger.getInstance()
      .recordOutput(
        "Odometry/pose",
        doubleArrayOf(
          odometryPose.x.inMeters, odometryPose.y.inMeters, odometryPose.rotation.inRadians
        )
      )
    Logger.getInstance()
      .recordOutput(
        "Odometry/targetPose",
        doubleArrayOf(
          targetPose.x.inMeters, targetPose.y.inMeters, targetPose.rotation.inRadians
        )
      )
  }

  private fun updateOdometry() {
    if (!(gyroInputs.gyroConnected)) {
      // in simulation we don't have access to a gyroscope, so when we command rotational movement
      // we have to calculate
      // the updated heading of the robot based on the output of what we've commanded (in real life
      // we can just read the heading from the gyro)
      val measuredStatesDifference = arrayOfNulls<SwerveModulePosition>(4)
      for (i in 0 until 4) {
        measuredStatesDifference[i] =
          SwerveModulePosition(
            (swerveModules[i].inputs.drivePosition - lastModulePositions[i]).inMeters,
            swerveModules[i].inputs.steeringPosition.inRotation2ds
          )
        lastModulePositions[i] = swerveModules[i].inputs.drivePosition
      }
      val positionDeltaTwist = swerveDriveKinematics.toTwist2d(*measuredStatesDifference)
      if (Constants.Tuning.SIMULATE_DRIFT) {
        undriftedPose = undriftedPose.exp(Twist2d(positionDeltaTwist))

        swerveDriveOdometry.resetPosition(
          gyroInputs.gyroYaw.inRotation2ds,
          swerveModules.map { it.modulePosition }.toTypedArray(),
          odometryPose.exp(
            Twist2d(
              positionDeltaTwist.dx.meters * 1.05,
              positionDeltaTwist.dy.meters * 1.05,
              positionDeltaTwist.dtheta.radians
            )
          )
            .pose2d
        )

        drift = undriftedPose.minus(odometryPose)

        Logger.getInstance().recordOutput("Odometry/undriftedPose", undriftedPose.pose2d)
      } else {
        odometryPose = odometryPose.exp(Twist2d(positionDeltaTwist))
      }

      gyroInputs.gyroYaw = odometryPose.rotation + gyroYawOffset
    } else {
      odometryPose =
        Pose2d(
          swerveDriveOdometry.update(
            gyroInputs.gyroYaw.inRotation2ds,
            swerveModules.map { it.modulePosition }.toTypedArray()
          )
        )
    }
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
    val swerveModuleStates: Array<SwerveModuleState>
    var desiredChassisSpeeds: ChassisSpeeds

    if (fieldOriented) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          driveVector.first, driveVector.second, angularVelocity, gyroInputs.gyroYaw
        )
    } else {
      desiredChassisSpeeds =
        ChassisSpeeds(
          driveVector.first,
          driveVector.second,
          angularVelocity,
        )
    }

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      val velocityTransform =
        Transform2d(
          Translation2d(
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vx,
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vy
          ),
          Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.omega
        )

      val twistToNextPose: Twist2d = velocityTransform.log()

      desiredChassisSpeeds =
        ChassisSpeeds(
          (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
        )
    }

    swerveModuleStates =
      swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds.chassisSpeedsWPILIB)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
    )

    setPointStates = swerveModuleStates.toMutableList()

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
    val velSwerveModuleStates: Array<SwerveModuleState>?
    val accelSwerveModuleStates: Array<SwerveModuleState>?
    if (fieldOriented) {
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVector.first, driveVector.second, angularVelocity, gyroInputs.gyroYaw
          )
            .chassisSpeedsWPILIB
        )
      accelSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisAccels.fromFieldRelativeAccels(
            driveAcceleration.first,
            driveAcceleration.second,
            angularAcceleration,
            gyroInputs.gyroYaw
          )
            .chassisAccelsWPILIB
        )
    } else {
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds(driveVector.first, driveVector.second, angularVelocity)
            .chassisSpeedsWPILIB
        )
      accelSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisAccels(driveAcceleration.first, driveAcceleration.second, angularAcceleration)
            .chassisAccelsWPILIB
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
    if (gyroInputs.gyroConnected) {
      swerveDriveOdometry.resetPosition(
        toAngle.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        odometryPose.pose2d
      )
    } else {
      gyroYawOffset = toAngle - undriftedPose.rotation
    }
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
