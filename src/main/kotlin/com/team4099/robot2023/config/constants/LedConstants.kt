package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.drivers.BlinkinLedDriver as Blinkin

class LedConstants {
  enum class LEDMode {
    IDLE,
    ITEM,
    FORWARD_TO_LEVEL,
    BACKWARD_TO_LEVEL,
    OUTTAKE,
    AUTO,
    TELEOP,
    DISABLED
  }

  enum class BlinkenMode(val blinkenMode: Blinkin.BlinkinLedMode) {
    IDLE(Blinkin.BlinkinLedMode.SOLID_ORANGE),
    ALERT(Blinkin.BlinkinLedMode.SOLID_DARK_RED),
    ITEM(Blinkin.BlinkinLedMode.SOLID_AQUA),
    FORWARD_TO_LEVEL(Blinkin.BlinkinLedMode.SOLID_AQUA),
    BACKWARD_TO_LEVEL(Blinkin.BlinkinLedMode.SOLID_AQUA),
    OUTTAKE(Blinkin.BlinkinLedMode.SOLID_AQUA),
    AUTO(Blinkin.BlinkinLedMode.SOLID_AQUA),
    TELEOP(Blinkin.BlinkinLedMode.SOLID_AQUA),
    DISABLED(Blinkin.BlinkinLedMode.SOLID_AQUA)
  }

  enum class CandleMode(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    IDLE(null, 0, 0, 0),
    ITEM(null, 0, 0, 0),
    FORWARD_TO_LEVEL(null, 0, 0, 0),
    BACKWARD_TO_LEVEL(null, 0, 0, 0),
    OUTTAKE(null, 0, 0, 0),
    AUTO(null, 0, 0, 0),
    TELEOP(null, 0, 0, 0),
    DISABLED(null, 0, 0, 0)
  }

  enum class SimMode(val color: Color8Bit) {
    IDLE(Color8Bit(Color.kOrange)),
    ITEM(Color8Bit(Color.kAqua)),
    POS_LEVELED(Color8Bit(Color.kAqua)),
    NEG_LEVELED(Color8Bit(Color.kAqua)),
    OUTTAKE(Color8Bit(Color.kAqua)),
    AUTO(Color8Bit(Color.kAqua)),
    TELEOP(Color8Bit(Color.kAqua)),
    DISABLED(Color8Bit(Color.kAqua))
  }
}
