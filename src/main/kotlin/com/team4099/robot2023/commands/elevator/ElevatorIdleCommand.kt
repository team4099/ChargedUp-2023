package com.team4099.robot2022.commands.intake

import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ElevatorIdleCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  override fun initialize() {
    elevator.desiredState = elevator.currentState.correspondingDesiredState
  }

  override fun execute() {
    elevator.setOpenLoop(0.0)
    Logger.getInstance().recordOutput("ActiveCommands/ElevatorIdleCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
