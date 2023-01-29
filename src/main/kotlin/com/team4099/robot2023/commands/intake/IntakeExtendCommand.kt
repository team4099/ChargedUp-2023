package com.team4099.robot2023.commands.intake

import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.intake.Intake
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class IntakeExtendCommand(val intake: Intake) : CommandBase() {

  private val lowerIntakeArm =
    SequentialCommandGroup(
      intake.rotateArmPosition(IntakeConstants.armStates.INTAKE.position),
      intake.holdArmPosition()
    )
  init {
    addRequirements(intake)
  }
  override fun initialize() {
    lowerIntakeArm.initialize()
  }

  override fun execute() {
    lowerIntakeArm.execute()
    intake.setRollerPower(IntakeConstants.rollerStates.INTAKE.velocity)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
