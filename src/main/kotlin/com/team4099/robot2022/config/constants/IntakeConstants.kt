package com.team4099.robot2022.config.constants

import com.team4099.lib.units.base.seconds

object IntakeConstants {
  val RAMP_TIME = 0.5
  val INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.9.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds
  const val TAB = "Intake"
  const val SUPPLY_CURRENT_LIMIT = 25.0
  const val SENSOR_CPR = 2048
  const val GEAR_RATIO = (12.0 / 36.0)

  enum class RollerState(val speed: Double) {
    IDLE(-0.1), // change to -0.1
    IN(0.9),
    OUT(-1.0)
  }

  enum class ArmState(val out: Boolean) {
    OUT(true),
    IN(false)
  }
}
