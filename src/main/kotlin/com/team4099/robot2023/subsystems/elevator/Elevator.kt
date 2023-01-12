package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.logging.TunableNumber
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.units.base.Length

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()
  val elevatorFeedForward =
    ElevatorFeedforward(
      ElevatorConstants.ELEVATOR_KS,
      ElevatorConstants.ELEVATOR_KG,
      ElevatorConstants.ELEVATOR_KV,
      ElevatorConstants.ELEVATOR_KA
    )

  private val kP = TunableNumber("Elevator/kP", ElevatorConstants.KP)
  private val kI = TunableNumber("Elevator/kI", ElevatorConstants.KI)
  private val kD = TunableNumber("Elevator/kD", ElevatorConstants.KD)

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition > ElevatorConstants.ELEVATOR_MAX_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition < ElevatorConstants.ELEVATOR_MAX_RETRACTION

  // Iterate through all desired states and see if the current position is equivalent to any of the
  // actual positions. If not, return that it's between two positions.
  val currentState: ElevatorConstants.ActualElevatorStates
    get() {
      for (desiredState in ElevatorConstants.DesiredElevatorStates.values()) {
        if ((desiredState.height - inputs.elevatorPosition).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) {
          return ElevatorConstants.ActualElevatorStates.fromDesiredState(desiredState)
        }
      }
      // TODO figure out if we'll ever need to know if we're between two states
      return ElevatorConstants.ActualElevatorStates.BETWEEN_TWO_STATES
    }

  init {}

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  fun setOpenLoop(percentOutput: Double) {
    io.setOpenLoop(percentOutput)

    if ((forwardLimitReached || reverseLimitReached) && percentOutput != 0.0) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }
  }

  fun setPosition(height: Length) {
    io.setPosition(height)
  }
}
