package com.team4099.robot2022.commands.intake

import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.intake.Intake
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ReverseCubeCommand(val intake: Intake) : CommandBase() {
  init {
    addRequirements(intake)
  }

  override fun initialize() {

    intake.rollerState = IntakeConstants.RollerState.CUBE_OUT
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ReverseCobeCommand", true)
  }

  override fun end(interrupted: Boolean) {
    intake.rollerState = IntakeConstants.RollerState.IDLE
  }

  override fun isFinished(): Boolean {
    return false
  }
}
