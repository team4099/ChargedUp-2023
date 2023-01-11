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
      LEDMode.IDLE -> setBlinken(BlinkenMode.IDLE.blinkenMode)
      LEDMode.ALERT -> setBlinken(BlinkenMode.ALERT.blinkenMode)
      LEDMode.ITEM -> setBlinken(BlinkenMode.ITEM.blinkenMode)
      LEDMode.NOT_LEVELED -> setBlinken(BlinkenMode.NOT_LEVELED.blinkenMode)
      LEDMode.OUTTAKE -> setBlinken(BlinkenMode.OUTTAKE.blinkenMode)
      LEDMode.AUTO -> setBlinken(BlinkenMode.AUTO.blinkenMode)
      LEDMode.TELEOP -> setBlinken(BlinkenMode.TELEOP.blinkenMode)
      LEDMode.DISABLED -> setBlinken(BlinkenMode.DISABLED.blinkenMode)
    }
  }

  private fun setBlinken(state: BlinkinLedDriver.BlinkinLedMode) {
    ledController.setMode(state)
  }
}
