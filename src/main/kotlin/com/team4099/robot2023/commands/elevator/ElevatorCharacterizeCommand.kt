package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.meters
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
class ElevatorCharacterizeCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  var hasMoved = false

  var appliedVolts = ElevatorConstants.ELEVATOR_KG
  var step = 0.001.volts

  override fun execute() {
    elevator.setOutputVoltage(appliedVolts)

    if (elevator.inputs.elevatorVelocity > 0.0.meters.perSecond) {
      hasMoved = true
      println(appliedVolts.inVolts - ElevatorConstants.ELEVATOR_KG.inVolts)
    }

    appliedVolts += step
  }

  override fun isFinished(): Boolean {
    return hasMoved
  }
}
