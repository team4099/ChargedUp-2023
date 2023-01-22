package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ElevatorCharacterizeCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  var hasMoved = false

  var appliedVolts = ElevatorConstants.ELEVATOR_KG
  var step = 0.001

  override fun execute() {
    elevator.setOutputVoltage(appliedVolts)

    if (elevator.inputs.elevatorVelocity > 0.0.meters.perSecond) {
      hasMoved = true
      println(appliedVolts.inVolts - ElevatorConstants.ELEVATOR_KG.inVolts)
    }

    appliedVolts += step.volts
  }

  override fun isFinished(): Boolean {
    return hasMoved
  }
}
