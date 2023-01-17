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
      LEDMode.ITEM -> setBlinken(BlinkenMode.ITEM.blinkenMode)
      LEDMode.FORWARD_TO_LEVEL -> setBlinken(BlinkenMode.FORWARD_TO_LEVEL.blinkenMode)
      LEDMode.BACKWARD_TO_LEVEL -> setBlinken(BlinkenMode.BACKWARD_TO_LEVEL.blinkenMode)
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
