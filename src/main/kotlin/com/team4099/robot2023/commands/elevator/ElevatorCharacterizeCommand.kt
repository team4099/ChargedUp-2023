package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ElevatorCharacterizeCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  var hasMoved = false
  var appliedVolts = 2.3.volts
  var step = 0.001

  override fun execute() {
    val batVoltage: Double
    if (RobotBase.isReal()) {
      batVoltage = LoggedPowerDistribution.getInstance().inputs.pdpVoltage
    } else {
      batVoltage = RoboRioSim.getVInVoltage()
    }
    elevator.setOpenLoop(appliedVolts.inVolts / batVoltage)

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
