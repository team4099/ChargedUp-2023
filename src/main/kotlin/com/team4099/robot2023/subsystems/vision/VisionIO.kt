package com.team4099.robot2023.subsystems.vision

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface VisionIO {
  class VisionInputs : LoggableInputs {
    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  fun updateInputs(inputs: VisionInputs) {}
}
