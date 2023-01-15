package com.team4099.robot2022.commands.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter

class SetStateElevatorCommand(val elevator: Elevator) : CommandBase() {

  lateinit var elevatorProfile: TrapezoidProfile<Meter>

  init {
    addRequirements(elevator)
  }

  constructor(elevator: Elevator, state: ElevatorConstants.DesiredElevatorStates) : this(elevator) {
    elevator.desiredState = state
  }

  override fun initialize() {
    // might be necessary if we change constraints in other command
    elevator.elevatorConstraints =
      TrapezoidProfile.Constraints(
        ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
      )

    elevator.startTime = Clock.fpgaTime
  }

  override fun execute() {
    elevator.setPosition()
    Logger.getInstance().recordOutput("ActiveCommands/OpenLoopRetractElevatorCommand", true)
  }
  override fun isFinished(): Boolean {
    return elevatorProfile.isFinished((Clock.fpgaTime - elevator.startTime))
  }
}
