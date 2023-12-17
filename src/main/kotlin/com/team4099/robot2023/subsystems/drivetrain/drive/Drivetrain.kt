package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.PoseEstimator
import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
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
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Supplier
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  val gyroNotConnectedAlert =
    Alert(
      "Gyro is not connected, field relative driving will be significantly worse.",
      Alert.AlertType.ERROR
    )

  val gyroInputs = GyroIO.GyroIOInputs()
  val swerveModules = swerveModuleIOs.getSwerveModules() // FL, FR, BL, BR

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

  var canMoveSafely = Supplier { false }

  var elevatorHeightSupplier = Supplier<Length> { 0.0.inches }

  var velocityTarget = 0.degrees.perSecond

  var targetedDriveVector = Pair(0.meters.perSecond, 0.meters.perSecond)

  var fieldOrientation = true // true denotes that when driving, it'll be field oriented.

  var targetedChassisSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)

  var targetedChassisAccels = edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)

  var currentState: DrivetrainState = DrivetrainState.UNINITIALIZED

  var currentRequest: DrivetrainRequest = DrivetrainRequest.ZeroSensors()
    set(value) {
      when (value) {
        is DrivetrainRequest.OpenLoop -> {
          velocityTarget = value.angularVelocity
          targetedDriveVector = value.driveVector
          fieldOrientation = value.fieldOriented
        }
        is DrivetrainRequest.ClosedLoop -> {
          targetedChassisSpeeds = value.chassisSpeeds
          targetedChassisAccels = value.chassisAccels
        }
        else -> {}
      }
      field = value
    }

  init {

    // Wheel speeds
    zeroSteering()
  }

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

  val moduleTranslations =
    listOf(
      frontLeftWheelLocation,
      frontRightWheelLocation,
      backLeftWheelLocation,
      backRightWheelLocation
    )

  val swerveDriveKinematics =
    SwerveDriveKinematics(
      frontLeftWheelLocation.translation2d,
      frontRightWheelLocation.translation2d,
      backLeftWheelLocation.translation2d,
      backRightWheelLocation.translation2d
    )

  var swerveDrivePoseEstimator = PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0001))

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
    get() = swerveDrivePoseEstimator.getLatestPose()
    //    get() {
    //      return Pose2d(
    //        42.875.inches + 79.centi.meters + 14.inches + 2.75.inches,
    //        113.25.inches + 1.inches,
    //        180.degrees
    //      )
    //    }
    set(value) {
      swerveDrivePoseEstimator.resetPose(value)

      if (RobotBase.isReal()) {} else {
        undriftedPose = odometryPose
        swerveModules.map { it.inputs.drift = 0.0.meters } // resetting drift to 0
      }
    }

  var rawGyroAngle = odometryPose.rotation

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

  var fieldVelocity = Velocity2d(0.0.meters.perSecond, 0.0.meters.perSecond)

  var robotVelocity = Pair(0.0.meters.perSecond, 0.0.meters.perSecond)

  var omegaVelocity = 0.0.radians.perSecond

  var lastModulePositions =
    mutableListOf(
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition()
    )

  var lastGyroYaw = 0.0.radians

  override fun periodic() {
    val startTime = Clock.realTimestamp

    odometryLock.lock(); // Prevents odometry updates while reading data

    gyroIO.updateInputs(gyroInputs)
    swerveModules.forEach { it.updateInputs() }

    odometryLock.unlock()

    gyroNotConnectedAlert.set(!gyroInputs.gyroConnected)

    swerveModules.forEach { it.periodic() }

    Logger.recordOutput(
      "Drivetrain/maxSetpointMetersPerSecond",
      DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
    )

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
    fieldVelocity = Velocity2d(fieldVelCalculated.x.perSecond, fieldVelCalculated.y.perSecond)

    omegaVelocity = chassisState.omega
    if (!gyroInputs.gyroConnected) {
      gyroInputs.gyroYawRate = omegaVelocity
      rawGyroAngle += Constants.Universal.LOOP_PERIOD_TIME * gyroInputs.gyroYawRate
      gyroInputs.gyroYaw = rawGyroAngle + gyroYawOffset
    }

    // updating odometry every loop cycle
    updateOdometry()

    Logger.recordOutput("Drivetrain/xVelocityMetersPerSecond", fieldVelocity.x.inMetersPerSecond)
    Logger.recordOutput("Drivetrain/yVelocityMetersPerSecond", fieldVelocity.y.inMetersPerSecond)

    Logger.processInputs("Drivetrain/Gyro", gyroInputs)
    Logger.recordOutput("Drivetrain/ModuleStates", *measuredStates)
    Logger.recordOutput("Drivetrain/SetPointStates", *setPointStates.toTypedArray())

    Logger.recordOutput(VisionConstants.POSE_TOPIC_NAME, odometryPose.pose2d)
    Logger.recordOutput(
      "Odometry/pose3d",
      Pose3d(
        odometryPose.x,
        odometryPose.y,
        1.0.meters,
        Rotation3d(gyroInputs.gyroRoll, gyroInputs.gyroPitch, gyroInputs.gyroYaw)
      )
        .pose3d
    )
    Logger.recordOutput(
      "Odometry/targetPose",
      doubleArrayOf(targetPose.x.inMeters, targetPose.y.inMeters, targetPose.rotation.inRadians)
    )

    Logger.recordOutput(
      "LoggedRobot/Subsystems/DrivetrainLoopTimeMS",
      (Clock.realTimestamp - startTime).inMilliseconds
    )

    var nextState = currentState

    when (currentState) {
      DrivetrainState.UNINITIALIZED -> {
        // Transitions
        nextState = DrivetrainState.ZEROING_SENSORS
      }
      DrivetrainState.ZEROING_SENSORS -> {
        zeroSensors()

        // Transitions
        currentRequest = DrivetrainRequest.Idle()
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.OPEN_LOOP -> {
        // Outputs
        setOpenLoop(velocityTarget, targetedDriveVector, fieldOrientation)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.CLOSED_LOOP -> {
        // Outputs
        setClosedLoop(targetedChassisSpeeds, targetedChassisAccels)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.IDLE -> {
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState

    // Log the current state
    Logger.recordOutput("Drivetrain/requestedState", currentState.toString())
  }

  private fun updateOdometry() {

    var deltaCount =
      if (gyroInputs.gyroConnected) gyroInputs.odometryYawPositions.size else Integer.MAX_VALUE
    for (i in 0..4) {
      deltaCount = Math.min(deltaCount, swerveModules[i].positionDeltas.size)
    }
    for (deltaIndex in 0..deltaCount) {
      // Read wheel deltas from each module
      val wheelDeltas = arrayOfNulls<SwerveModulePosition>(4)
      for (moduleIndex in 0..4) {
        wheelDeltas[moduleIndex] = swerveModules[moduleIndex].positionDeltas[deltaIndex]
      }

      var driveTwist = swerveDriveKinematics.toTwist2d(*wheelDeltas)

      if (gyroInputs.gyroConnected) {
        driveTwist =
          edu.wpi.first.math.geometry.Twist2d(
            driveTwist.dx, driveTwist.dy, (gyroInputs.rawGyroYaw - lastGyroYaw).inRadians
          )
        lastGyroYaw = gyroInputs.rawGyroYaw
      }

      swerveDrivePoseEstimator.addDriveData(Clock.fpgaTime.inSeconds, Twist2d(driveTwist))
      odometryPose = odometryPose.exp(Twist2d(driveTwist))
    }

    // reversing the drift to store the ground truth pose
    if (RobotBase.isSimulation() && Constants.Tuning.SIMULATE_DRIFT) {
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
      swerveDriveOdometry.update((gyroInputs.gyroYaw).inRotation2ds, undriftedModules)

      drift = undriftedPose.minus(odometryPose)

      Logger.recordOutput(VisionConstants.SIM_POSE_TOPIC_NAME, undriftedPose.pose2d)
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
    val flipDrive = if (FMSData.allianceColor == DriverStation.Alliance.Red) -1 else 1
    val allianceFlippedDriveVector =
      Pair(driveVector.first * flipDrive, driveVector.second * flipDrive)

    val swerveModuleStates: Array<SwerveModuleState>
    var desiredChassisSpeeds: ChassisSpeeds

    if (fieldOriented) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
          angularVelocity,
          odometryPose.rotation
        )
    } else {
      desiredChassisSpeeds =
        ChassisSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
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
            driveVector.first, driveVector.second, angularVelocity, odometryPose.rotation
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
            odometryPose.rotation
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

    SwerveDriveKinematics.desaturateWheelSpeeds(
      velSwerveModuleStates, DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond
    )

    setPointStates = velSwerveModuleStates.toMutableList()

    // Once we have all of our states obtained for both velocity and acceleration, apply these
    // states to each swerve module
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionClosedLoop(
        velSwerveModuleStates[moduleIndex], accelSwerveModuleStates[moduleIndex]
      )
    }
  }

  fun setClosedLoop(
    chassisSpeeds: edu.wpi.first.math.kinematics.ChassisSpeeds,
    chassisAccels: edu.wpi.first.math.kinematics.ChassisSpeeds =
      edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
  ) {
    var chassisSpeeds = chassisSpeeds

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      val velocityTransform =
        Transform2d(
          Translation2d(
            Constants.Universal.LOOP_PERIOD_TIME *
              chassisSpeeds.vxMetersPerSecond.meters.perSecond,
            Constants.Universal.LOOP_PERIOD_TIME *
              chassisSpeeds.vyMetersPerSecond.meters.perSecond
          ),
          Constants.Universal.LOOP_PERIOD_TIME *
            chassisSpeeds.omegaRadiansPerSecond.radians.perSecond
        )

      val twistToNextPose: Twist2d = velocityTransform.log()

      chassisSpeeds =
        ChassisSpeeds(
          (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
        )
          .chassisSpeedsWPILIB
    }

    val velSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    val accelSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisAccels)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      velSwerveModuleStates, DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond
    )

    setPointStates = velSwerveModuleStates.toMutableList()

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
    zeroGyroPitch(0.0.degrees)
    zeroGyroRoll()
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

    swerveDrivePoseEstimator.resetPose(Pose2d(odometryPose.x, odometryPose.y, gyroInputs.gyroYaw))

    if (RobotBase.isSimulation()) {
      swerveDriveOdometry.resetPosition(
        toAngle.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        undriftedPose.pose2d
      )
    }

    if (!(gyroInputs.gyroConnected)) {
      gyroYawOffset = toAngle - rawGyroAngle
    }
  }

  fun zeroGyroPitch(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroPitch(toAngle)
  }

  fun zeroGyroRoll(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroRoll(toAngle)
  }

  /** Zeros the steering motors for each swerve module. */
  fun zeroSteering() {
    swerveModules.forEach { it.zeroSteering() }
  }

  /** Zeros the drive motors for each swerve module. */
  private fun zeroDrive() {
    swerveModules.forEach { it.zeroDrive() }
  }

  fun addVisionData(visionData: List<PoseEstimator.TimestampedVisionUpdate>) {
    swerveDrivePoseEstimator.addVisionData(visionData)
  }

  companion object {
    // Drivetrain multithreading
    var odometryLock: Lock = ReentrantLock()
    fun setOdometryLock(Locked: Boolean) {
      if (Locked) {
        odometryLock.lock()
      } else {
        odometryLock.unlock()
      }
    }

    // Drivetrain states for state machine.
    enum class DrivetrainState {
      UNINITIALIZED,
      IDLE,
      ZEROING_SENSORS,
      OPEN_LOOP,
      CLOSED_LOOP;

      inline fun equivalentToRequest(request: Request.DrivetrainRequest): Boolean {
        return (
          (request is DrivetrainRequest.ZeroSensors && this == ZEROING_SENSORS) ||
            (request is DrivetrainRequest.OpenLoop && this == OPEN_LOOP) ||
            (request is DrivetrainRequest.ClosedLoop && this == CLOSED_LOOP) ||
            (request is DrivetrainRequest.Idle && this == IDLE)
          )
      }
    }

    inline fun fromRequestToState(request: Request.DrivetrainRequest): DrivetrainState {
      return when (request) {
        is DrivetrainRequest.OpenLoop -> DrivetrainState.OPEN_LOOP
        is DrivetrainRequest.ClosedLoop -> DrivetrainState.CLOSED_LOOP
        is DrivetrainRequest.ZeroSensors -> DrivetrainState.ZEROING_SENSORS
        is DrivetrainRequest.Idle -> DrivetrainState.IDLE
      }
    }
  }
}
