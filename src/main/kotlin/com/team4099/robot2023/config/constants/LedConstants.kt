package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import org.team4099.lib.drivers.BlinkinLedDriver as Blinkin

class LedConstants {
  enum class LEDMode {
    IDLE,
    ALERT
  }

  enum class BlinkenMode(val blinkenMode: Blinkin.BlinkinLedMode) {
    IDLE(Blinkin.BlinkinLedMode.SOLID_ORANGE),
    ALERT(Blinkin.BlinkinLedMode.SOLID_DARK_RED)
  }

  enum class CandleMode(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    IDLE(null, 0, 0, 0),
    ALERT(StrobeAnimation(255, 0, 0), 0, 0, 0),
    ITEM(null, 0, 0, 0),
    NOT_LEVELED(null, 0, 0, 0),
    OUTTAKE(null, 0, 0, 0),

    AUTO(null, 0, 0, 0),
    TELEOP(null, 0, 0, 0),
    DISABLED(null, 0, 0, 0)
  }
}
