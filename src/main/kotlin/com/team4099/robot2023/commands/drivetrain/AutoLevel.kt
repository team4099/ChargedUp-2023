package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

class AutoLevel(val drivetrain: Drivetrain) : CommandBase() {
  private val gyroPID: ProfiledPIDController<Radian, Velocity<Meter>>

  var alignmentAngle = 0.0.degrees

  val gyroAngle: Angle
    get() {
      return when (alignmentAngle.absoluteValue.inDegrees % 180.0) {
        0.0 -> drivetrain.gyroInputs.gyroPitch
        90.0 -> drivetrain.gyroInputs.gyroRoll
        else -> 0.0.degrees
      }
    }

  // TODO update falconutils to allow degrees / mps as a pid controller
  val levelkP =
    LoggedTunableValue(
      "Drivetrain/AutoLevel/levelkP",
      DrivetrainConstants.PID.AUTO_LEVEL_KP,
    )
  val levelkI =
    LoggedTunableValue(
      "Drivetrain/AutoLevel/levelkI",
      DrivetrainConstants.PID.AUTO_LEVEL_KI,
    )
  val levelkD =
    LoggedTunableValue(
      "Drivetrain/AutoLevel/levelkD",
      DrivetrainConstants.PID.AUTO_LEVEL_KD,
    )

  val maxVelocity =
    LoggedTunableValue(
      "Drivetrain/AutoLevel/maxVelocity", DrivetrainConstants.PID.AUTO_LEVEL_MAX_VEL_SETPOINT
    )

  val maxAccel =
    LoggedTunableValue(
      "Drivetrain/AutoLevel/maxAccel", DrivetrainConstants.PID.AUTO_LEVEL_MAX_ACCEL_SETPOINT
    )

  val gyroFeedback: Pair<Value<Fraction<Meter, Second>>, Value<Fraction<Meter, Second>>>
    get() {
      var feedback =
        gyroPID.calculate(gyroAngle, DrivetrainConstants.DOCKING_GYRO_SETPOINT) *
          (alignmentAngle + 0.1.degrees).sign
      // 0.1 so sign of 0 degrees returns 1

      return when (alignmentAngle.absoluteValue.inDegrees % 180.0) {
        0.0 -> Pair(feedback, 0.0.meters.perSecond)
        90.0 -> Pair(0.0.meters.perSecond, feedback)
        else -> Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
      }
    }

  init {
    addRequirements(drivetrain)

    // TODO(might want sperate pid controller for pitch vs roll)

    gyroPID =
      ProfiledPIDController(
        levelkP.get(),
        levelkI.get(),
        levelkD.get(),
        TrapezoidProfile.Constraints(maxVelocity.get(), maxAccel.get())
      )

    alignmentAngle = drivetrain.closestAlignmentAngle
  }

  override fun initialize() {
    gyroPID.reset(gyroAngle) // maybe do first for x?
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/AutoLevelCommand", true)

    drivetrain.setOpenLoop(0.0.radians.perSecond, gyroFeedback, fieldOriented = true)

    Logger.getInstance()
      .recordOutput(
        "AutoLevel/DesiredPitchDegrees", DrivetrainConstants.DOCKING_GYRO_SETPOINT.inDegrees
      )
    Logger.getInstance()
      .recordOutput("AutoLevel/CurrentPitchDegrees", drivetrain.gyroInputs.gyroPitch.inDegrees)
    Logger.getInstance()
      .recordOutput("AutoLevel/CorrectionVelocityX", gyroFeedback.first.inMetersPerSecond)
    Logger.getInstance()
      .recordOutput("AutoLevel/CorrectionVelocityY", gyroFeedback.second.inMetersPerSecond)
  }

  var lastTimeSinceUnbalanced = Clock.fpgaTime

  override fun isFinished(): Boolean {

    var balanced =
      (gyroAngle - DrivetrainConstants.DOCKING_GYRO_SETPOINT).absoluteValue <
        DrivetrainConstants.DOCKING_GYRO_TOLERANCE

    if (!balanced) {
      lastTimeSinceUnbalanced = Clock.fpgaTime
    }

    return (Clock.fpgaTime - lastTimeSinceUnbalanced > DrivetrainConstants.DOCKING_TIME_THRESHOLD)
  }

  override fun end(interrupted: Boolean) {
    drivetrain.setOpenLoop(0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond))
  }
}
