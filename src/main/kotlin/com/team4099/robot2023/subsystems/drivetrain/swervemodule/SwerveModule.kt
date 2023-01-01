package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotation2ds
import com.team4099.lib.units.derived.inVoltsPerMeters
import com.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import com.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import com.team4099.lib.units.derived.inVoltsPerRadian
import com.team4099.lib.units.derived.inVoltsPerRadianSeconds
import com.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import com.team4099.lib.units.derived.perMeterPerSecond
import com.team4099.lib.units.derived.perMeterPerSecondPerSecond
import com.team4099.lib.units.derived.perRadian
import com.team4099.lib.units.derived.perRadianPerSecond
import com.team4099.lib.units.derived.perRadianSeconds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.littletonrobotics.junction.Logger
import kotlin.math.IEEErem
import kotlin.math.withSign

class SwerveModule(val io: SwerveModuleIO) {
  val inputs = SwerveModuleIO.SwerveModuleIOInputs()

  var modulePosition = SwerveModulePosition()

  private var speedSetPoint: LinearVelocity = 0.feet.perSecond
  private var accelerationSetPoint: LinearAcceleration = 0.feet.perSecond.perSecond

  private var steeringSetPoint: Angle = 0.degrees

  private var shouldInvert = false

  private val steeringkP =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkP",
      DrivetrainConstants.PID.STEERING_KP,
      Pair({ it.inVoltsPerRadian }, { it.volts.perRadian })
    )
  private val steeringkI =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkI",
      DrivetrainConstants.PID.STEERING_KI,
      Pair({ it.inVoltsPerRadianSeconds }, { it.volts.perRadianSeconds })
    )
  private val steeringkD =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkD",
      DrivetrainConstants.PID.STEERING_KD,
      Pair({ it.inVoltsPerRadiansPerSecond }, { it.volts.perRadianPerSecond })
    )

  private val steeringMaxVel =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringMaxVelRadPerSec", DrivetrainConstants.STEERING_VEL_MAX
    )
  private val steeringMaxAccel =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringMaxAccelRadPerSecSq", DrivetrainConstants.STEERING_ACCEL_MAX
    )

  private val drivekP =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekP",
      DrivetrainConstants.PID.DRIVE_KP,
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )

  private val drivekI =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekI",
      DrivetrainConstants.PID.DRIVE_KI,
      Pair({ it.inVoltsPerMeters }, { it.volts / (1.meters.perSecond * 1.seconds) })
    )

  private val drivekD =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekD",
      DrivetrainConstants.PID.DRIVE_KD,
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  init {
    if (isReal()) {
      steeringkP.initDefault(DrivetrainConstants.PID.STEERING_KP)
      steeringkI.initDefault(DrivetrainConstants.PID.STEERING_KI)
      steeringkD.initDefault(DrivetrainConstants.PID.STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.DRIVE_KD)
    } else {
      steeringkP.initDefault(DrivetrainConstants.PID.SIM_STEERING_KP)
      steeringkI.initDefault(DrivetrainConstants.PID.SIM_STEERING_KI)
      steeringkD.initDefault(DrivetrainConstants.PID.SIM_STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KD)
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    // Updating SwerveModulePosition every loop cycle
    modulePosition.distanceMeters = inputs.drivePosition.inMeters
    modulePosition.angle = inputs.steeringPosition.inRotation2ds

    if (steeringkP.hasChanged() || steeringkI.hasChanged() || steeringkD.hasChanged()) {
      io.configureSteeringPID(steeringkP.get(), steeringkI.get(), steeringkD.get())
    }

    if (steeringMaxVel.hasChanged() || steeringMaxAccel.hasChanged()) {
      io.configureSteeringMotionMagic(steeringMaxVel.get(), steeringMaxAccel.get())
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

    //    io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
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

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using open loop control.
   *
   * @param desiredState The desired SwerveModuleState. Contains desired angle as well as X and Y
   * velocities
   */
  fun setPositionOpenLoop(desiredState: SwerveModuleState, optimize: Boolean = true) {
    if (optimize) {
      val optimizedState =
        SwerveModuleState.optimize(desiredState, inputs.steeringPosition.inRotation2ds)
      io.setOpenLoop(
        optimizedState.angle.angle,
        optimizedState
          .speedMetersPerSecond // consider desaturating wheel speeds here if it doesn't work
        // from drivetrain
      )
    } else {
      io.setOpenLoop(desiredState.angle.angle, desiredState.speedMetersPerSecond)
    }
  }

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using feed forward.
   *
   * @param desiredVelState The desired SwerveModuleState. Contains desired angle as well as X and Y
   * velocities
   * @param desiredAccelState The desired SwerveModuleState that contains desired acceleration
   * vectors.
   * @param optimize Whether velocity and acceleration vectors should be optimized (if possible)
   */
  fun setPositionClosedLoop(
    desiredVelState: SwerveModuleState,
    desiredAccelState: SwerveModuleState,
    optimize: Boolean = true
  ) {
    if (optimize) {
      val optimizedVelState =
        SwerveModuleState.optimize(desiredVelState, inputs.steeringPosition.inRotation2ds)
      val optimizedAccelState =
        SwerveModuleState.optimize(desiredAccelState, inputs.steeringPosition.inRotation2ds)
      io.setClosedLoop(
        optimizedVelState.angle.angle,
        optimizedVelState.speedMetersPerSecond.meters.perSecond,
        optimizedAccelState.speedMetersPerSecond.meters.perSecond.perSecond
      )
    } else {
      io.setClosedLoop(
        desiredVelState.angle.angle,
        desiredVelState.speedMetersPerSecond.meters.perSecond,
        desiredAccelState.speedMetersPerSecond.meters.perSecond.perSecond
      )
    }
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
