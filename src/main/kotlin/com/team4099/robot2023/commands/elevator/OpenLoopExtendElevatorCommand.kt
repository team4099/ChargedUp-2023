package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.volts

class OpenLoopExtendElevatorCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  override fun execute() {
    elevator.setOutputVoltage(12.volts)
    Logger.getInstance().recordOutput("ActiveCommands/OpenLoopExtendElevatorCommand", true)
  }

  override fun isFinished(): Boolean {
    return false
  }
}
