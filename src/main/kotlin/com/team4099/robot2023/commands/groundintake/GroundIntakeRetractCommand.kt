package com.team4099.robot2023.commands.groundintake

import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class GroundIntakeRetractCommand(val groundIntake: GroundIntake) : CommandBase() {

  private val retractIntakeArm =
    SequentialCommandGroup(
      groundIntake.rotateGroundIntakeToAngle(GroundIntakeConstants.ArmStates.STOWED.position),
      groundIntake.holdArmPosition()
    )
  init {
    addRequirements(groundIntake)
  }
  override fun initialize() {
    retractIntakeArm.initialize()
  }

  override fun execute() {
    retractIntakeArm.execute()
    groundIntake.setRollerVoltage(GroundIntakeConstants.RollerStates.IDLE.voltage)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
