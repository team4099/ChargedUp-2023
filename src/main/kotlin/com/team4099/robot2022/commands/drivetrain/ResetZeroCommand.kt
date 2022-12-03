package com.team4099.robot2022.commands.drivetrain

import com.team4099.robot2022.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ResetZeroCommand(val drivetrain: Drivetrain) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetModuleZero()
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetZeroCommand", true)
  }
}
