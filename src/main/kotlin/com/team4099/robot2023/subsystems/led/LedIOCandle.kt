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
      LEDMode.ITEM -> setCANdleState(CandleMode.ITEM)
      LEDMode.POS_LEVELED -> setCANdleState(CandleMode.POS_LEVELED)
      LEDMode.NEG_LEVELED -> setCANdleState(CandleMode.NEG_LEVELED)
      LEDMode.OUTTAKE -> setCANdleState(CandleMode.OUTTAKE)
      LEDMode.AUTO -> setCANdleState(CandleMode.AUTO)
      LEDMode.TELEOP -> setCANdleState(CandleMode.TELEOP)
      LEDMode.DISABLED -> setCANdleState(CandleMode.DISABLED)
    }
  }

  private fun setCANdleState(state: CandleMode) {
    if (state.animation == null) {
      ledController.setLEDs(state.r, state.g, state.b)
    } else {
      ledController.animate(state.animation)
    }
  }
}
