package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

/**
 * This command is used to experimentally determine the kS value for the intake It prints the first
 * voltage level that causes the elevator to move
 *
 * @property hasMoved used as end condition
 * @property appliedVolts the current amount of volts being applied to the motors
 * @property step the increase in volts per iteration
 */
class GroundIntakeCharacterizeCommand(val groundIntake: GroundIntake) : CommandBase() {

  init {
    addRequirements(groundIntake)
  }

  var hasMoved = false
  var appliedVolts = 0.volts

  var sim_step = 0.01.volts
  var real_step = 0.1.volts

  var moveTolerance = 0.1.degrees.perSecond

  override fun initialize() {
    hasMoved = false
    appliedVolts = 0.volts
  }

  override fun execute() {
    groundIntake.io.setArmVoltage(appliedVolts)

    if ((groundIntake.inputs.armVelocity - 0.0.degrees.perSecond).absoluteValue > moveTolerance) {
      hasMoved = true
      println(appliedVolts.inVolts)
      Logger.getInstance().recordOutput("GroundIntake/CharacterizationOutput", appliedVolts.inVolts)
    }

    if (RobotBase.isReal()) {
      appliedVolts += real_step
    } else {
      appliedVolts += sim_step
    }
  }

  override fun isFinished(): Boolean {
    return hasMoved
  }
}
