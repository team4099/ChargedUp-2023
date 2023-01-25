package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class OpenLoopExtendElevatorCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  override fun initialize() {
    elevator.desiredState = ElevatorConstants.DesiredElevatorStates.MAX_HEIGHT
  }

  override fun execute() {
    elevator.setOpenLoop(1.0)
    Logger.getInstance().recordOutput("ActiveCommands/OpenLoopExtendElevatorCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
