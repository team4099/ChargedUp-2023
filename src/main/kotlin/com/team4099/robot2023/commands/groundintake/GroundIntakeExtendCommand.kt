package com.team4099.robot2023.commands.groundintake

import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class GroundIntakeExtendCommand(val groundIntake: GroundIntake) : CommandBase() {

  private val lowerIntakeArm =
    SequentialCommandGroup(
      groundIntake.rotateGroundIntakeToAngle(GroundIntakeConstants.ArmStates.INTAKE.position),
      groundIntake.holdArmPosition()
    )
  init {
    addRequirements(groundIntake)
  }
  override fun initialize() {
    lowerIntakeArm.initialize()
  }

  override fun execute() {
    lowerIntakeArm.execute()
    groundIntake.setRollerVoltage(GroundIntakeConstants.RollerStates.INTAKE.voltage)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
