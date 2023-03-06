package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LedConstants
import org.littletonrobotics.junction.Logger

class Led(val io: LedIO) {
  var inputs = LedIO.LedIOInputs()
  var state = LedConstants.LEDMode.IDLE
    set(value) {
      io.setState(value)
      field = value
    }

  init {
    state = state
  }

  fun periodic() {
    io.updateInputs(inputs)
    Logger.getInstance().processInputs("LED", inputs)
    Logger.getInstance().recordOutput("LED/state", state.name)
  }
}
