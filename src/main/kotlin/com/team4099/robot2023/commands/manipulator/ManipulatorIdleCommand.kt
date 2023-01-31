package com.team4099.robot2022.commands.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

/**
 * This command is used to set the idle speed based off of last manipulator roller state if last
 * roller state was cone in, then idle state would be cone idle if last roller state was cube in,
 * then idle state would be cube idle
 */
class ManipulatorIdleCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  override fun initialize() {
    manipulator.lastRollerRunTime = Clock.fpgaTime
  }

  override fun execute() {
    var idleState = ManipulatorConstants.RollerStates.NO_SPIN
    if (manipulator.lastRollerState.voltage.sign ==
      ManipulatorConstants.RollerStates.CONE_IN.voltage.sign
    ) {
      idleState = ManipulatorConstants.RollerStates.CONE_IDLE
    } else if (manipulator.lastRollerState.voltage.sign ==
      ManipulatorConstants.RollerStates.CUBE_IN.voltage.sign
    ) {
      idleState = ManipulatorConstants.RollerStates.CUBE_IDLE
    }

    Logger.getInstance().recordOutput("/Manipulator/idleState", idleState.name)
    manipulator.setRollerPower(idleState.voltage)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
