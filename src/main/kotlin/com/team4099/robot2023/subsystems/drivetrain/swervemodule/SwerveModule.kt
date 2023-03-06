package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterPerSecondPerSecond
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
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
      "Drivetrain/moduleSteeringkP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  private val steeringkI =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkI",
      Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val steeringkD =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
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
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )

  private val drivekI =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekI",
      Pair({ it.inVoltsPerMeters }, { it.volts / (1.meters.perSecond * 1.seconds) })
    )

  private val drivekD =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekD",
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
          .speedMetersPerSecond
          .meters
          .perSecond // consider desaturating wheel speeds here if it doesn't work
        // from drivetrain
      )
      Logger.getInstance()
        .recordOutput("${io.label}/steeringSetpoint", optimizedState.angle.degrees)
    } else {
      io.setOpenLoop(desiredState.angle.angle, desiredState.speedMetersPerSecond.meters.perSecond)
      Logger.getInstance().recordOutput("${io.label}/steeringSetpoint", desiredState.angle.degrees)
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
      Logger.getInstance().recordOutput("Drivetrain/desiredVelState", optimizedVelState.speedMetersPerSecond)
      Logger.getInstance().recordOutput("Drivetrain/desiredAngle", optimizedVelState.angle.degrees)
      Logger.getInstance().recordOutput("Drivetrain/desiredAccelState", optimizedAccelState.speedMetersPerSecond)
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
    io.setDriveBrakeMode(brake)
  }

  fun setSteeringBrakeMode(brake: Boolean) {
    io.setSteeringBrakeMode(brake)
  }
}
