package com.team4099.robot2022.commands.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.CommandBase

class IntakeCubeCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  override fun initialize() {
    manipulator.lastRollerRunTime = Clock.fpgaTime
  }

  override fun execute() {
    manipulator.setRollerPower(ManipulatorConstants.RollerStates.CUBE_IN.voltage)
  }

  override fun end(interrupted: Boolean) {
    manipulator.lastRollerState = manipulator.rollerState
  }

  override fun isFinished(): Boolean {
    return manipulator.hasCube
  }
}
