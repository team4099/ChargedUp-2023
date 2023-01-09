package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation


class LedConstants {
  enum class LEDMode {
    IDLE,
    ALERT
  }

  enum class BlinkenMode(val blinkenMode: BlinkenDriver.BlinkenLedMode) {
    IDLE(BlinkenDriver.BlinkenLedMode.SOLID_ORANGE),
    ALERT(BlinkenDriver.BlinkenLedMode.SOLID_DARK_RED)
  }

  enum class CandleMode(val animation: Animation?, val r: Int, val g: Int, val b: Int){
    IDLE(null, 0, 0, 0),
    ALERT(StrobeAnimation(255, 0, 0), 0, 0, 0)
  }
}
