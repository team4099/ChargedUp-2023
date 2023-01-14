package com.team4099.robot2022.commands.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds

class SetStateElevatorCommand(val elevator: Elevator) : CommandBase() {

  lateinit var elevatorProfile: TrapezoidProfile<Meter>

  var startTime = Clock.fpgaTime

  init {
    addRequirements(elevator)
  }

  constructor(elevator: Elevator, state: ElevatorConstants.DesiredElevatorStates) : this(elevator) {
    elevator.desiredState = state
  }

  override fun initialize() {
    elevatorProfile =
      TrapezoidProfile(
        elevator.elevatorConstraints,
        TrapezoidProfile.State(elevator.desiredState.height, 0.0.meters / 1.0.seconds),
        TrapezoidProfile.State(
          elevator.inputs.elevatorPosition, elevator.inputs.elevatorVelocity
        )
      )

    startTime = Clock.fpgaTime
  }

  override fun execute() {
    elevator.setHeight(elevatorProfile.calculate((Clock.fpgaTime - startTime)))

    Logger.getInstance().recordOutput("ActiveCommands/OpenLoopRetractElevatorCommand", true)
  }
  override fun isFinished(): Boolean {
    return elevatorProfile.isFinished((Clock.fpgaTime - startTime))
  }
}
