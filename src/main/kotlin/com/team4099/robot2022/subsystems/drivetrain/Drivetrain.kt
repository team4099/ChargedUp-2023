package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.geometry.Pose
import com.team4099.lib.geometry.Translation
import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inches
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
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
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

  /**
   * Sets the drivetrain to the specified angular and X & Y velocities based on the current angular
   * and linear acceleration. Calculates both angular and linear velocities and acceleration and
   * calls set for each SwerveModule object.
   *
   * @param angularVelocity The angular velocity of a specified drive
   * @param driveVector.first The linear velocity on the X axis
   * @param driveVector.second The linear velocity on the Y axis
   * @param angularAcceleration The angular acceleration of a specified drive
   * @param driveAcceleration.first The linear acceleration on the X axis
   * @param driveAcceleration.second The linear acceleration on the Y axis
   */
  fun set(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    fieldOriented: Boolean = true,
    angularAcceleration: AngularAcceleration = 0.0.radians.perSecond.perSecond,
    driveAcceleration: Pair<LinearAcceleration, LinearAcceleration> =
      Pair(0.0.meters.perSecond.perSecond, 0.0.meters.perSecond.perSecond)
  ) {

    // Logger.addEvent("Drivetrain", "setting with $driveVector and $angularVelocity")
    //    Logger.addEvent("Drivetrain", "gyro angle: ${(-gyroAngle).inDegrees}")
    val vX =
      if (fieldOriented) {
        driveVector.first * (-inputs.gyroYaw).cos - driveVector.second * (-inputs.gyroYaw).sin
      } else {
        driveVector.first
      }
    val vY =
      if (fieldOriented) {
        driveVector.second * (-inputs.gyroYaw).cos + driveVector.first * (-inputs.gyroYaw).sin
      } else {
        driveVector.second
      }

    val aY =
      if (fieldOriented) {
        driveAcceleration.second * (-inputs.gyroYaw).cos +
          driveAcceleration.first * (-inputs.gyroYaw).sin
      } else {
        driveAcceleration.second
      }
    val aX =
      if (fieldOriented) {
        driveAcceleration.first * (-inputs.gyroYaw).cos -
          driveAcceleration.second * (-inputs.gyroYaw).sin
      } else {
        driveAcceleration.first
      }

    val a = vX + angularVelocity * DrivetrainConstants.DRIVETRAIN_LENGTH / 2
    val b = vX - angularVelocity * DrivetrainConstants.DRIVETRAIN_LENGTH / 2
    val c = vY + angularVelocity * DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    val d = vY - angularVelocity * DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    // Logger.addEvent("Drivetrain", "vX: $vX, angular velocity: $angularVelocity")

    wheelSpeeds[0] = hypot(b, d)
    wheelSpeeds[1] = hypot(b, c)
    wheelSpeeds[2] = hypot(a, d)
    wheelSpeeds[3] = hypot(a, c)

    val aA =
      aX +
        (angularAcceleration.value * DrivetrainConstants.DRIVETRAIN_LENGTH.value / 2)
          .inches
          .perSecond
          .perSecond
    val aB =
      aX -
        (angularAcceleration.value * DrivetrainConstants.DRIVETRAIN_LENGTH.value / 2)
          .inches
          .perSecond
          .perSecond
    val aC =
      aY +
        (angularAcceleration.value * DrivetrainConstants.DRIVETRAIN_WIDTH.value / 2)
          .inches
          .perSecond
          .perSecond
    val aD =
      aY -
        (angularAcceleration.value * DrivetrainConstants.DRIVETRAIN_WIDTH.value / 2)
          .inches
          .perSecond
          .perSecond

    wheelAccelerations[0] = kotlin.math.hypot(aB.value, aD.value).feet.perSecond.perSecond
    wheelAccelerations[1] = kotlin.math.hypot(aB.value, aC.value).feet.perSecond.perSecond
    wheelAccelerations[2] = kotlin.math.hypot(aA.value, aD.value).feet.perSecond.perSecond
    wheelAccelerations[3] = kotlin.math.hypot(aA.value, aC.value).feet.perSecond.perSecond

    val maxWheelSpeed = wheelSpeeds.maxOrNull()
    if (maxWheelSpeed != null && maxWheelSpeed > DrivetrainConstants.DRIVE_SETPOINT_MAX) {
      for (i in 0 until DrivetrainConstants.WHEEL_COUNT) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxWheelSpeed.inMetersPerSecond
      }
    }
    wheelAngles[0] = atan2(b, d)
    wheelAngles[1] = atan2(b, c)
    wheelAngles[2] = atan2(a, d)
    wheelAngles[3] = atan2(a, c)
    //    Logger.addEvent("Drivetrain", "wheel angle: $wheelAngles")

    swerveModules[0].set(wheelAngles[0], wheelSpeeds[0], wheelAccelerations[0])
    swerveModules[1].set(wheelAngles[1], wheelSpeeds[1], wheelAccelerations[1])
    swerveModules[2].set(wheelAngles[2], wheelSpeeds[2], wheelAccelerations[2])
    swerveModules[3].set(wheelAngles[3], wheelSpeeds[3], wheelAccelerations[3])
  }

  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    fieldOriented: Boolean = true
  ) {
    // Logger.addEvent("Drivetrain", "setting open loop with $driveVector and $angularVelocity")

    val vX =
      if (fieldOriented) {
        driveVector.first * (-inputs.gyroYaw).cos - driveVector.second * (-inputs.gyroYaw).sin
      } else {
        driveVector.first
      }
    val vY =
      if (fieldOriented) {
        driveVector.second * (-inputs.gyroYaw).cos + driveVector.first * (-inputs.gyroYaw).sin
      } else {
        driveVector.second
      }

    if (vX == 0.0.meters.perSecond &&
      vY == 0.0.meters.perSecond &&
      angularVelocity == 0.0.degrees.perSecond
    ) {
      wheelSpeeds[0] = 0.0.meters.perSecond
      wheelSpeeds[1] = 0.0.meters.perSecond
      wheelSpeeds[2] = 0.0.meters.perSecond
      wheelSpeeds[3] = 0.0.meters.perSecond
    } else {
      val a = vX + angularVelocity * DrivetrainConstants.DRIVETRAIN_LENGTH / 2
      val b = vX - angularVelocity * DrivetrainConstants.DRIVETRAIN_LENGTH / 2
      val c = vY + angularVelocity * DrivetrainConstants.DRIVETRAIN_WIDTH / 2
      val d = vY - angularVelocity * DrivetrainConstants.DRIVETRAIN_WIDTH / 2

      wheelSpeeds[0] = hypot(b, d)
      wheelSpeeds[1] = hypot(b, c)
      wheelSpeeds[2] = hypot(a, d)
      wheelSpeeds[3] = hypot(a, c)

      val maxWheelSpeed = wheelSpeeds.maxOrNull()
      if (maxWheelSpeed != null && maxWheelSpeed > DrivetrainConstants.DRIVE_SETPOINT_MAX) {
        for (i in 0 until DrivetrainConstants.WHEEL_COUNT) {
          wheelSpeeds[i] =
            wheelSpeeds[i] / maxWheelSpeed.inMetersPerSecond *
            DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
        }
      }
      wheelAngles[0] = atan2(b, d)
      wheelAngles[1] = atan2(b, c)
      wheelAngles[2] = atan2(a, d)
      wheelAngles[3] = atan2(a, c)
    }

    swerveModules[0].setOpenLoop(
      wheelAngles[0], wheelSpeeds[0] / DrivetrainConstants.DRIVE_SETPOINT_MAX
    )
    swerveModules[1].setOpenLoop(
      wheelAngles[1], wheelSpeeds[1] / DrivetrainConstants.DRIVE_SETPOINT_MAX
    )
    swerveModules[2].setOpenLoop(
      wheelAngles[2], wheelSpeeds[2] / DrivetrainConstants.DRIVE_SETPOINT_MAX
    )
    swerveModules[3].setOpenLoop(
      wheelAngles[3], wheelSpeeds[3] / DrivetrainConstants.DRIVE_SETPOINT_MAX
    )
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
