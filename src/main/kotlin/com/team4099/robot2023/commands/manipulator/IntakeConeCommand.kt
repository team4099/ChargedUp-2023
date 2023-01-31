package com.team4099.robot2022.commands.intake

import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.CommandBase

/**
 * This command is used to set the roller speed to intake cone Ends once manipulator detects it has
 * a cone
 */
class IntakeConeCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  override fun initialize() {}

  override fun execute() {
    manipulator.setRollerPower(ManipulatorConstants.RollerStates.CONE_IN.voltage)
  }

  override fun end(interrupted: Boolean) {
    manipulator.lastRollerState = ManipulatorConstants.RollerStates.CONE_IN
  }

  override fun isFinished(): Boolean {
    return manipulator.hasCone
  }
}
