package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LedConstants
import com.team4099.robot2023.config.constants.LedConstants.LEDMode
import com.team4099.robot2023.config.constants.LedConstants.SimMode
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

object LedIOSim : LedIO {
  private val ledWidget = Mechanism2d(400.0, 400.0, Color8Bit(Color.kWhite))
  private var state: LEDMode = LEDMode.IDLE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = state.name
  }

  override fun setState(newState: LEDMode) {
    state = newState

    when (state) {
      else -> LedConstants.BlinkenMode.IDLE
    }
  }

  private fun setWidget(color: SimMode) {
    ledWidget.setBackgroundColor(color.color)
  }
}
