package com.team4099.robot2022.commands.elevator

import com.team4099.robot2022.subsytems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ElevatorIdleCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  override fun execute() {
    elevator.setOpenLoop(0.0)

    Logger.getInstance().recordOutput("ActiveCommands/ElevatorIdleCommand", true)
  }
}
