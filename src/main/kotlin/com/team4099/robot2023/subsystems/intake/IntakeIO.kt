package com.team4099.robot2023.subsystems.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {

    override fun toLog(table: LogTable?) {}

    override fun fromLog(table: LogTable?) {}
  }

  fun updateInputs(inputs: IntakeIOInputs) {}
}
