package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.inInches
import com.team4099.lib.units.base.inches
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/** Interface for all hardware implementations for the Elevator subsystem */
interface ElevatorIO {
  /** Defines and logs all inputs that are required for all hardware implementations. */
  class ElevatorIOInputs : LoggableInputs {

    var position = 0.0.inches

    override fun toLog(table: LogTable?) {
      table?.put("positionInches", position.inInches)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("positionInches", position.inInches)?.let { position = it.inches }
    }
  }

  /**
   * Method to be implemented to update data encapsulated by `inputs` object
   * @param inputs Input data retrieved from hardware implementation and used by subsystem logic
   * class
   */
  fun updateInputs(inputs: ElevatorIOInputs) {}

  /**
   * Runs elevator motors at a specified power
   * @param percentOutput Ratio that represents the amount of applied power
   */
  fun setOpenLoop(percentOutput: Double) {}
}
