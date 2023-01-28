package com.team4099.robot2023.commands.intake

import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.intake.Intake
import edu.wpi.first.wpilibj2.command.CommandBase

class IntakeIdleCommand(val intake: Intake) : CommandBase() {
  init {
    addRequirements(intake)
  }
  override fun initialize() {
    intake.rollerState = IntakeConstants.ROLLER_STATE.IDLE
    intake.arm
  }

  override fun execute() {}
  override fun isFinished(): Boolean {
    return false
  }
}
