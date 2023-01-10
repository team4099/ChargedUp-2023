package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

class AutoLevel(val drivetrain: Drivetrain) : CommandBase() {
  private val pitchPID: ProfiledPIDController<Radian, Velocity<Meter>>

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

  init {
    addRequirements(drivetrain)

    pitchPID =
      ProfiledPIDController(
        levelkP.get(),
        levelkI.get(),
        levelkD.get(),
        TrapezoidProfile.Constraints(maxVelocity.get(), maxAccel.get())
      )
  }

  override fun initialize() {
    pitchPID.reset(drivetrain.gyroInputs.gyroPitch) // maybe do first for x?
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/AutoLevelCommand", true)

    val pitchFeedback = pitchPID.calculate(drivetrain.gyroInputs.gyroPitch, 0.0.degrees)

    drivetrain.setOpenLoop(
      0.0.radians.perSecond, Pair(pitchFeedback, 0.0.meters.perSecond), fieldOriented = true
    )

    Logger.getInstance().recordOutput("AutoLevel/DesiredPitchDegrees", 0.0)
    Logger.getInstance()
      .recordOutput("AutoLevel/CurrentPitchDegrees", drivetrain.gyroInputs.gyroPitch.inDegrees)
    Logger.getInstance()
      .recordOutput("AutoLevel/CorrectionVelocity", pitchFeedback.inMetersPerSecond)
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drivetrain.setOpenLoop(0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond))
  }
}
