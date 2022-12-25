package com.team4099.robot2023.config.constants

import com.team4099.lib.units.base.seconds

object FeederConstants {
  // feeder_power temp
  const val FEEDER_POWER = 1.0
  const val AUTO_FEEDER_POWER = 0.6
  const val TAB = "Feeder"

  const val FLOOR_SUPPLY_CURRENT_LIMIT = 25.0
  const val VERTICAL_SUPPLY_CURRENT_LIMIT = 30.0

  // temp values
  const val BEAM_BREAK_BROKEN_TIME = 0.03
  const val BEAM_BREAK_BACKWARDS_TIME = 0.05

  val BEAM_BREAK_THRESHOLD = 0.15.seconds
  const val BEAM_BREAK_FILTER_SIZE = 100

  // not final
  enum class FeederState(val floorMotorPower: Double, val verticalMotorPower: Double) {
    FORWARD_ALL(FEEDER_POWER, FEEDER_POWER),
    FORWARD_FLOOR(FEEDER_POWER, 0.0),
    BACKWARD_ALL(-FEEDER_POWER, -FEEDER_POWER),
    BACKWARD_FLOOR(-FEEDER_POWER, 0.0),
    BACKWARD_VERTICAL(0.0, -FEEDER_POWER),
    NEUTRAL(0.0, 0.0),
    SHOOT(FEEDER_POWER, FEEDER_POWER),
    AUTO_SHOOT(AUTO_FEEDER_POWER, AUTO_FEEDER_POWER)
  }
}
