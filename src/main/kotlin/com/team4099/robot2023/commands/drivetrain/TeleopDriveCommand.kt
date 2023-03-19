package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class TeleopDriveCommand(
  val driver: DriverProfile,
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
    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)
    drivetrain.setOpenLoop(rotation, speed)
    Logger.getInstance().recordOutput("ActiveCommands/TeleopDriveCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
