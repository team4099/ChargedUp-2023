package com.team4099.robot2023.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LedConstants.CandleMode
import com.team4099.robot2023.config.constants.LedConstants.LEDMode

object LedIOCandle : LedIO {

  private val ledController = CANdle(Constants.Led.LED_CANDLE_ID, Constants.Universal.CANIVORE_NAME)
  private var lastState: LEDMode = LEDMode.IDLE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastState.name
  }

  override fun setState(newState: LEDMode) {
    lastState = newState
    when (newState) {
      LEDMode.IDLE -> setCANdleState(CandleMode.IDLE)
      LEDMode.OUTTAKE -> setCANdleState(CandleMode.OUTTAKE)
      LEDMode.INTAKE -> setCANdleState(CandleMode.INTAKE)
      LEDMode.AUTO -> setCANdleState(CandleMode.AUTO)
      LEDMode.TELEOP -> setCANdleState(CandleMode.TELEOP)
      LEDMode.CUBE -> setCANdleState(CandleMode.CUBE)
      LEDMode.CONE -> setCANdleState(CandleMode.CONE)
      LEDMode.SINGLE_SUBSTATION -> setCANdleState(CandleMode.SINGLE_SUBSTATION)
      LEDMode.DOUBLE_SUBSTATION -> setCANdleState(CandleMode.DOUBLE_SUBSTATION)
      LEDMode.SCORE -> setCANdleState(CandleMode.SCORE)
      LEDMode.MOVEMENT -> setCANdleState(CandleMode.MOVEMENT)
    }
  }

  private fun setCANdleState(state: CandleMode) {
    if (state.animation == null) {
      if (state.address != null) {
        ledController.setLEDs(
          state.r, state.g, state.b, 255, state.address.first, state.address.second
        )
      } else {
        ledController.setLEDs(state.r, state.g, state.b)
      }
    } else {
      ledController.animate(state.animation)
    }
  }
}
