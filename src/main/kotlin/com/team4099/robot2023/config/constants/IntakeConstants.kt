package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.seconds

object IntakeConstants {
  // TODO: 1/9/23
  // make sure constants are their actual values
  val RAMP_TIME = 0

  // used to detect intake/outake, values need testing
  val INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.9.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  const val SENSOR_CPR = 42
  // waiting for deisgn
  const val GEAR_RATIO = 0

  enum class IntakeState(val speed: Double) {
    // TODO: 1/9/23
    // figure out why IDLE speed is -0.1 for rapid react

    IDLE(0.0),
    IN(0.9),
    OUT(-1.0)
  }
}
