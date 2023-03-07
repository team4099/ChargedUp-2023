package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.drivers.BlinkinLedDriver as Blinkin

class LedConstants {
  enum class LEDMode {
    IDLE,
    OUTTAKE,
    INTAKE,
    AUTO,
    TELEOP,
    CUBE,
    CONE,
    SINGLE_SUBSTATION,
    DOUBLE_SUBSTATION,
    MOVEMENT,
    SCORE
  }

  enum class BlinkenMode(val blinkenMode: Blinkin.BlinkinLedMode) {
    IDLE(Blinkin.BlinkinLedMode.SOLID_RED),
    OUTTAKE(Blinkin.BlinkinLedMode.SOLID_BLUE),
    INTAKE(Blinkin.BlinkinLedMode.SOLID_GREEN),
    AUTO(Blinkin.BlinkinLedMode.SOLID_GOLD),
    TELEOP(Blinkin.BlinkinLedMode.SOLID_WHITE),
    CUBE(Blinkin.BlinkinLedMode.SOLID_VIOLET),
    CONE(Blinkin.BlinkinLedMode.SOLID_YELLOW),
    SINGLE_SUBSTATION(Blinkin.BlinkinLedMode.FIXED_STROBE_WHITE),
    DOUBLE_SUBSTATION(Blinkin.BlinkinLedMode.FIXED_STROBE_WHITE),
    SCORE(Blinkin.BlinkinLedMode.FIXED_STROBE_WHITE),
    MOVEMENT(Blinkin.BlinkinLedMode.FIXED_STROBE_WHITE)
  }

  enum class CandleMode(
    val animation: Animation?,
    val r: Int,
    val g: Int,
    val b: Int,
    val address: Pair<Int, Int>?
  ) {
    IDLE(null, 255, 0, 0, null),
    OUTTAKE(null, 0, 0, 255, null),
    INTAKE(null, 64, 255, 51, null),
    AUTO(null, 255, 215, 0, null),
    TELEOP(null, 255, 255, 255, null),
    CUBE(null, 162, 25, 255, null),
    CONE(null, 0, 0, 0, null),
    SINGLE_SUBSTATION(StrobeAnimation(255, 255, 255), 0, 0, 0, Pair(0, 25)),
    DOUBLE_SUBSTATION(StrobeAnimation(255, 255, 255), 0, 0, 0, Pair(100, 25)),
    SCORE(StrobeAnimation(255, 255, 255), 0, 0, 0, Pair(50, 25)),
    MOVEMENT(StrobeAnimation(255, 255, 255), 0, 0, 0, null)
  }

  enum class SimMode(val color: Color8Bit) {
    IDLE(Color8Bit(Color.kOrange))
  }
}
