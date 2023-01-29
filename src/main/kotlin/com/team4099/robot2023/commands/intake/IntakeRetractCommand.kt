package com.team4099.robot2023.commands.intake

import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.intake.Intake
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class IntakeRetractCommand(val intake: Intake) : CommandBase() {

  private val retractIntakeArm =
    SequentialCommandGroup(
      intake.rotateArmPosition(IntakeConstants.armStates.STOWED.position),
      intake.holdArmPosition()
    )
  init {
    addRequirements(intake)
  }
  override fun initialize() {
    retractIntakeArm.initialize()
  }

  override fun execute() {
    retractIntakeArm.execute()
    intake.setRollerPower(IntakeConstants.rollerStates.IDLE.velocity)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
