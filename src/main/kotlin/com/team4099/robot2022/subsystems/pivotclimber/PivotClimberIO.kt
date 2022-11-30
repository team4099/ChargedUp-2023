package com.team4099.robot2022.subsystems.climber

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface PivotClimberIO {
  class PivotClimberIOInputs : LoggableInputs {
    var isExtended: Boolean = false

    override fun toLog(table: LogTable?) {
      table?.put("pivotExtended", isExtended)
    }

    override fun fromLog(table: LogTable?) {
      table?.getBoolean("pivotExtended", isExtended)?.let { isExtended = it }
    }
  }

  fun updateInputs(io: PivotClimberIOInputs)
}
