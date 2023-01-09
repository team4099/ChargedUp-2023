package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LedConstants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface LedIO {
  class LedIOInputs: LoggableInputs {
    var ledState = LedConstants.LEDMode.IDLE.name

    override fun toLog(table: LogTable?) {
      table?.put("ledState", ledState)
    }

    override fun fromLog(table: LogTable?) {
      table?.getString("ledState", ledState).let {ledState = it}
    }
  }

  fun setState(newState: LedConstants.LEDMode){}
}
