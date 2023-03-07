package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.LedConstants.BlinkenMode
import com.team4099.robot2023.config.constants.LedConstants.LEDMode
import org.team4099.lib.drivers.BlinkinLedDriver

object LedIOBlinken : LedIO {
  private val ledController = BlinkinLedDriver(Constants.Led.LED_BLINKEN_ID)
  private var lastLedState: LEDMode = LEDMode.IDLE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = lastLedState.name
  }

  override fun setState(newState: LEDMode) {
    lastLedState = newState
    when (newState) {
      LEDMode.IDLE -> setBlinken(BlinkenMode.IDLE)
      LEDMode.OUTTAKE -> setBlinken(BlinkenMode.OUTTAKE)
      LEDMode.INTAKE -> setBlinken(BlinkenMode.INTAKE)
      LEDMode.AUTO -> setBlinken(BlinkenMode.AUTO)
      LEDMode.TELEOP -> setBlinken(BlinkenMode.TELEOP)
      LEDMode.CUBE -> setBlinken(BlinkenMode.CUBE)
      LEDMode.CONE -> setBlinken(BlinkenMode.CONE)
      LEDMode.SINGLE_SUBSTATION -> setBlinken(BlinkenMode.SINGLE_SUBSTATION)
      LEDMode.DOUBLE_SUBSTATION -> setBlinken(BlinkenMode.DOUBLE_SUBSTATION)
      LEDMode.SCORE -> setBlinken(BlinkenMode.SCORE)
      LEDMode.MOVEMENT -> setBlinken(BlinkenMode.MOVEMENT)
      else -> setBlinken(BlinkenMode.IDLE)
    }
  }

  private fun setBlinken(state: BlinkenMode) {
    ledController.setMode(state.blinkenMode)
  }
}
