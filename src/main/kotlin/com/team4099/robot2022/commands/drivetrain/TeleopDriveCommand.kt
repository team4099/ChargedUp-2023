package com.team4099.robot2022.commands.drivetrain

import com.team4099.robot2022.config.constants.DrivetrainConstants
import com.team4099.robot2022.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import kotlin.math.sign

class TeleopDriveCommand(
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val robotOriented: () -> Boolean,
  val drivetrain: Drivetrain
) : CommandBase() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {}

  override fun execute() {
    val speed =
      Pair(
        DrivetrainConstants.DRIVE_SETPOINT_MAX * driveX() * driveX() * sign(driveX()),
        DrivetrainConstants.DRIVE_SETPOINT_MAX * driveY() * driveY() * sign(driveY())
      )
    val direction = DrivetrainConstants.TURN_SETPOINT_MAX * turn() * turn() * turn()
    drivetrain.setOpenLoop(direction, speed, fieldOriented = !robotOriented())
    Logger.getInstance().recordOutput("ActiveCommands/TeleopDriveCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
