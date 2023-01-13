package com.team4099.robot2022.commands.intake

import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class openLoopRetractElevatorCommand(val elevator: Elevator) : CommandBase() {
  init {
    addRequirements(elevator)
  }

  override fun initialize() {
    elevator.desiredState = ElevatorConstants.DesiredElevatorStates.MAX_HEIGHT
  }

  override fun execute() {
    elevator.setOpenLoop(-1.0)
    Logger.getInstance().recordOutput("ActiveCommands/OpenLoopRetractElevatorCommand", true)
  }
  override fun isFinished(): Boolean {
    return false
  }
}
