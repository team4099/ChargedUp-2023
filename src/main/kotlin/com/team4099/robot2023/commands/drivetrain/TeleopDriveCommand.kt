package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import kotlin.math.sign

class TeleopDriveCommand(
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain
) : CommandBase() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {}

  override fun execute() {
    val flipDrive = if (FMSData.allianceColor == DriverStation.Alliance.Red) -1 else 1
    val flipTurn = if (FMSData.allianceColor == DriverStation.Alliance.Red) -1 else 1

    val speedMultiplier = if (slowMode()) 0.25 else 1

    val speed =
      Pair(
        DrivetrainConstants.DRIVE_SETPOINT_MAX * speedMultiplier *
          driveX() *
          driveX() *
          sign(driveX()) *
          flipDrive,
        DrivetrainConstants.DRIVE_SETPOINT_MAX * speedMultiplier *
          driveY() *
          driveY() *
          sign(driveY()) *
          flipDrive
      )
    val direction = DrivetrainConstants.TURN_SETPOINT_MAX * turn() * turn() * turn() * turn() * turn() * flipTurn
    drivetrain.setOpenLoop(direction, speed)
    Logger.getInstance().recordOutput("ActiveCommands/TeleopDriveCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
