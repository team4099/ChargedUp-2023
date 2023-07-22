package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ZeroSensorsCommand(val drivetrain: Drivetrain) : CommandBase() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors()
  }

  override fun isFinished(): Boolean {
    return true
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ZeroSensorsCommand", true)
  }
}
