package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.util.Alert
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  val gyroNotConnectedAlert =
    Alert(
      "Gyro is not connected, field relative driving will be significantly worse.",
      Alert.AlertType.ERROR
    )

  val gyroInputs = GyroIO.GyroIOInputs()
  val swerveModules = swerveModuleIOs.getSwerveModules()
  var gyroYawOffset = 0.0.radians

  val closestAlignmentAngle: Angle
    get() {
      for (angle in -180..90 step 90) {
        if ((odometryPose.rotation - angle.degrees).absoluteValue <= 45.degrees) {
          return angle.degrees
        }
      }
      return 0.0.degrees
    }

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

  var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1)
  var visionStdDevs = VecBuilder.fill(1.0, 1.0, 1.0)

  var swerveDrivePoseEstimator =
    SwerveDrivePoseEstimator(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray(),
      Pose2d().pose2d, // TODO initialize this with the robot's actual starting pose
      stateStdDevs,
      visionStdDevs
    )

  var swerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray()
    )

  var setPointStates =
    mutableListOf(
      SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
    )

  var odometryPose: Pose2d
    get() = Pose2d(swerveDrivePoseEstimator.estimatedPosition)
    set(value) {
      swerveDrivePoseEstimator.resetPosition(
        gyroInputs.gyroYaw.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        value.pose2d
      )

      if (RobotBase.isReal()) {
        zeroGyroYaw(odometryPose.rotation)
      } else {
        undriftedPose = odometryPose
        swerveModules.map { it.inputs.drift = 0.0.meters } // resetting drift to 0
      }
    }

  var undriftedPose: Pose2d
    get() = Pose2d(swerveDriveOdometry.poseMeters)
    set(value) {
      swerveDriveOdometry.resetPosition(
        gyroInputs.gyroYaw.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        value.pose2d
      )
    }

  var targetPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, 0.0.radians)

  var drift: Transform2d = Transform2d(Translation2d(), 0.0.radians)

  var fieldVelocity = Pair(0.0.meters.perSecond, 0.0.meters.perSecond)

  var robotVelocity = Pair(0.0.meters.perSecond, 0.0.meters.perSecond)

  var omegaVelocity = 0.0.radians.perSecond

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
    val fieldVelCalculated =
      Translation2d(
        chassisState.vx.inMetersPerSecond.meters, chassisState.vy.inMetersPerSecond.meters
      )
        .rotateBy(odometryPose.rotation) // we don't use this but it's there if you want it ig

    robotVelocity = Pair(chassisState.vx, chassisState.vy)
    fieldVelocity = Pair(fieldVelCalculated.x.perSecond, fieldVelCalculated.y.perSecond)

    omegaVelocity = chassisState.omega
    if (!gyroInputs.gyroConnected) {
      gyroInputs.gyroYawRate = omegaVelocity
      gyroInputs.gyroYaw =
        gyroInputs.gyroYaw +
        Constants.Universal.LOOP_PERIOD_TIME * gyroInputs.gyroYawRate +
        gyroYawOffset
    }

    Logger.getInstance()
      .recordOutput("Drivetrain/xVelocityMetersPerSecond", fieldVelocity.first.inMetersPerSecond)
    Logger.getInstance()
      .recordOutput("Drivetrain/yVelocityMetersPerSecond", fieldVelocity.second.inMetersPerSecond)

    Logger.getInstance().processInputs("Drivetrain/Gyro", gyroInputs)
    Logger.getInstance().recordOutput("Drivetrain/ModuleStates", *measuredStates)
    Logger.getInstance().recordOutput("Drivetrain/SetPointStates", *setPointStates.toTypedArray())

    Logger.getInstance().recordOutput(VisionConstants.POSE_TOPIC_NAME, odometryPose.pose2d)
    Logger.getInstance()
      .recordOutput(
        "Odometry/pose3d",
        Pose3d(
          odometryPose.x,
          odometryPose.y,
          1.0.meters,
          Rotation3d(gyroInputs.gyroRoll, gyroInputs.gyroPitch, gyroInputs.gyroYaw)
        )
          .pose3d
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
    // reversing the drift to store the ground truth pose
    if (!(RobotBase.isReal()) && Constants.Tuning.SIMULATE_DRIFT) {
      val undriftedModules = arrayOfNulls<SwerveModulePosition>(4)
      for (i in 0 until 4) {
        undriftedModules[i] =
          SwerveModulePosition(
            (
              swerveModules[i].modulePosition.distanceMeters.meters -
                swerveModules[i].inputs.drift
              )
              .inMeters,
            swerveModules[i].modulePosition.angle
          )
      }
      swerveDriveOdometry.update(gyroInputs.gyroYaw.inRotation2ds, undriftedModules)

      drift = undriftedPose.minus(odometryPose)

      Logger.getInstance().recordOutput(VisionConstants.SIM_POSE_TOPIC_NAME, undriftedPose.pose2d)
    }

    swerveDrivePoseEstimator.update(
      gyroInputs.gyroYaw.inRotation2ds, swerveModules.map { it.modulePosition }.toTypedArray()
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
      // Getting velocity and acceleration states from the drive & angular velocity vectors and
      // drive & angular acceleration vectors (respectively)
      // This is with respect to the field, meaning all velocity and acceleration vectors are
      // adjusted to be relative to the field instead of relative to the robot.
      velSwerveModuleStates =
        swerveDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVector.first, driveVector.second, angularVelocity, gyroInputs.gyroYaw
          )
            .chassisSpeedsWPILIB
        )

      // Although this isn't perfect, calculating acceleration states using the same math as
      // velocity can get us "good enough" accel states to minimize skew
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
      // Getting velocity and acceleration states from the drive & angular velocity vectors and
      // drive & angular acceleration vectors (respectively)
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

    // Once we have all of our states obtained for both velocity and acceleration, apply these
    // states to each swerve module
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
      swerveDrivePoseEstimator.resetPosition(
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
