package com.team4099.robot2023.subsystems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ElevatorIO {
  class ElevatorInputs : LoggableInputs {
    override fun toLog(table: LogTable) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable) {
      TODO("Not yet implemented")

    }
  }

  fun updateInputs(inputs: ElevatorInputs) {}

}
