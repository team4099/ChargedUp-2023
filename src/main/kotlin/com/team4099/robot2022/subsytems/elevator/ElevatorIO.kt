package com.team4099.robot2022.subsytems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/** Interface for all hardware implementations for the Elevator subsystem */
interface ElevatorIO {
  /** Defines and logs all inputs that are required for all hardware implementations. */
  class ElevatorIOInputs : LoggableInputs {
    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  /**
   * Method to be implemented to update data encapsulated by `inputs` object
   * @param inputs Input data retrieved from hardware implementation and used by subsystem logic
   * class
   */
  fun updateInputs(inputs: ElevatorIOInputs) {}
}
