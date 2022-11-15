package com.team4099.robot2022.subsytems.elevator

import org.littletonrobotics.junction.AutoLog

/** Interface for all hardware implementations for the Elevator subsystem */
interface ElevatorIO {
  /**
   * Defines and logs all inputs that are required for all hardware implementations.
   *
   * @AutoLog Annotation that automatically implements `toLog()` and `fromLog()` for us. For more
   * info visit
   * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/NEW-FOR-2023.md#autolog-annotation
   */
  @AutoLog class ElevatorIOInputs

  /**
   * Method to be implemented to update data encapsulated by `inputs` object
   * @param inputs Input data retrieved from hardware implementation and used by subsystem logic
   * class
   */
  fun updateInputs(inputs: ElevatorIOInputs) {}
}
