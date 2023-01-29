package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.subsystems.intake.Intake
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

/**
 * This command is used to experimentally determine the kS value for the elevator It prints the
 * first voltage level that causes the elevator to move
 *
 * @property hasMoved used as end condition
 * @property appliedVolts the current amount of volts being applied to the motors
 * @property step the increase in volts per iteration
 */
class IntakeCharacterizeCommand(val intake: Intake) : CommandBase() {

  init {
    addRequirements(intake)
  }

  var hasMoved = false
  var appliedVolts = 0.volts
  var step = 0.001.volts

  override fun execute() {
    intake.io.setArmVoltage(appliedVolts)

    if (intake.inputs.armVelocity > 0.0.degrees.perSecond) {
      hasMoved = true
      println(appliedVolts.inVolts)
    }

    appliedVolts += step
  }

  override fun isFinished(): Boolean {
    return hasMoved
  }
}
