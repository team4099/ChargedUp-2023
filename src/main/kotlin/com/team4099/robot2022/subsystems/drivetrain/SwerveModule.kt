package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotation2ds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.inRadiansPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveModulePosition
import org.littletonrobotics.junction.Logger
import kotlin.math.IEEErem
import kotlin.math.withSign

class SwerveModule(val io: SwerveModuleIO) {
  val inputs = SwerveModuleIO.SwerveModuleIOInputs()

  var position: SwerveModulePosition =
    SwerveModulePosition(
      inputs.driveVelocity.inMetersPerSecond, inputs.steeringPosition.inRotation2ds
    )

  private var speedSetPoint: LinearVelocity = 0.feet.perSecond
  private var accelerationSetPoint: LinearAcceleration = 0.feet.perSecond.perSecond

  private var steeringSetPoint: Angle = 0.degrees

  private var shouldInvert = false

  private val steeringkP =
    TunableNumber("Drivetrain/moduleSteeringkP", DrivetrainConstants.PID.STEERING_KP)
  private val steeringkI =
    TunableNumber("Drivetrain/moduleSteeringkI", DrivetrainConstants.PID.STEERING_KI)
  private val steeringkD =
    TunableNumber("Drivetrain/moduleSteeringkD", DrivetrainConstants.PID.STEERING_KD)

  private val steeringMaxVel =
    TunableNumber(
      "Drivetrain/moduleSteeringMaxVelRadPerSec",
      DrivetrainConstants.STEERING_VEL_MAX.inRadiansPerSecond
    )
  private val steeringMaxAccel =
    TunableNumber(
      "Drivetrain/moduleSteeringMaxAccelRadPerSecSq",
      DrivetrainConstants.STEERING_ACCEL_MAX.inRadiansPerSecondPerSecond
    )

  private val drivekP = TunableNumber("Drivetrain/moduleDrivekP", DrivetrainConstants.PID.DRIVE_KP)
  private val drivekI = TunableNumber("Drivetrain/moduleDrivekI", DrivetrainConstants.PID.DRIVE_KI)
  private val drivekD = TunableNumber("Drivetrain/moduleDrivekD", DrivetrainConstants.PID.DRIVE_KD)

  init {}

  fun periodic() {
    io.updateInputs(inputs)

    if (steeringkP.hasChanged() || steeringkI.hasChanged() || steeringkD.hasChanged()) {
      io.configureSteeringPID(steeringkP.get(), steeringkI.get(), steeringkD.get())
    }

    if (steeringMaxVel.hasChanged() || steeringMaxAccel.hasChanged()) {
      io.configureSteeringMotionMagic(
        steeringMaxVel.get().radians.perSecond,
        steeringMaxAccel.get().radians.perSecond.perSecond
      )
    }

    if (drivekP.hasChanged() || drivekI.hasChanged() || drivekD.hasChanged()) {
      io.configureDrivePID(drivekP.get(), drivekI.get(), drivekD.get())
    }

    Logger.getInstance().processInputs(io.label, inputs)
    //    Logger.getInstance()
    //      .recordOutput(
    //        "${io.label}/driveSpeedSetpointMetersPerSecond",
    //        if (!shouldInvert) speedSetPoint.inMetersPerSecond
    //        else -speedSetPoint.inMetersPerSecond
    //      )
    //    Logger.getInstance()
    //      .recordOutput(
    //        "${io.label}/driveAccelSetpointMetersPerSecondSq",
    //        accelerationSetPoint.inMetersPerSecondPerSecond
    //      )
    //    Logger.getInstance()
    //      .recordOutput("${io.label}/steeringSetpointDegrees", steeringSetPoint.inDegrees)
    //    Logger.getInstance()
    //      .recordOutput(
    //        "${io.label}/steeringValueDegreesWithMod",
    //        inputs.steeringPosition.inDegrees.IEEErem(360.0)
    //      )
  }

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using feed forward.
   *
   * @param steering The angular position desired for the swerve module to be set to
   * @param speed The speed desired for the swerve module to reach
   * @param acceleration The linear acceleration used to calculate how to reach the desired speed
   */
  fun set(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration = 0.0.meters.perSecond.perSecond,
    optimize: Boolean = true
  ) {
    if (speed == 0.feet.perSecond) {
      io.setOpenLoop(steeringSetPoint, 0.0)
      return
    }
    var steeringDifference =
      (steering - inputs.steeringPosition).inRadians.IEEErem(2 * Math.PI).radians

    shouldInvert = steeringDifference.absoluteValue > (Math.PI / 2).radians && optimize
    if (shouldInvert) {
      steeringDifference -= Math.PI.withSign(steeringDifference.inRadians).radians
    }

    speedSetPoint =
      if (shouldInvert) {
        speed * -1
      } else {
        speed
      }
    accelerationSetPoint =
      if (shouldInvert) {
        acceleration * -1
      } else {
        acceleration
      }
    steeringSetPoint = inputs.steeringPosition + steeringDifference
    //    driveFalcon.set(speedSetPoint / DrivetrainConstants.DRIVE_SETPOINT_MAX)

    io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
  }

  fun setOpenLoop(steering: Angle, speed: Double, optimize: Boolean = true) {
    var steeringDifference =
      (steering - inputs.steeringPosition).inRadians.IEEErem(2 * Math.PI).radians

    shouldInvert = steeringDifference.absoluteValue > (Math.PI / 2).radians && optimize
    if (shouldInvert) {
      steeringDifference -= Math.PI.withSign(steeringDifference.inRadians).radians
    }

    val outputPower =
      if (shouldInvert) {
        speed * -1
      } else {
        speed
      }
    steeringSetPoint = inputs.steeringPosition + steeringDifference
    io.setOpenLoop(steeringSetPoint, outputPower)
  }

  /** Creates event of the current potentiometer value as needs to be manually readjusted. */
  fun resetModuleZero() {
    io.resetModuleZero()
  }

  /** Zeros the steering motor */
  fun zeroSteering() {
    io.zeroSteering()
  }

  /** Zeros the drive motor */
  fun zeroDrive() {
    io.zeroDrive()
  }

  fun setDriveBrakeMode(brake: Boolean) {
    io.setBrakeMode(brake)
  }
}
