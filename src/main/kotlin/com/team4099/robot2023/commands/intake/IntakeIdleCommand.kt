package com.team4099.robot2022.commands.intake

import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class IntakeIdleCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  override fun initialize() {
    manipulator.rollerState = ManipulatorConstants.RollerState.IDLE
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/IntakeIdleCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
