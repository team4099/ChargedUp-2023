package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts

object ManipulatorConstants {
  // TODO: 1/9/23
  // make sure constants are their actual values
  val RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  val INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.9.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  val MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  const val SUPPLY_CURRENT_LIMIT = 25
  const val SENSOR_CPR = 42.0
  // waiting for deisgn
  const val GEAR_RATIO = 1.0

  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps

  // TODO(figure out what voltage compensation should be)
  val VOLTAGE_COMPENSATION = 10.0.volts

  enum class RollerState(val speed: Double) {
    // TODO: 1/9/23
    // figure out why IDLE speed is -0.1 for rapid react

    IDLE(0.0),
    CONE_IN(0.8),
    CUBE_IN(0.9),
    CONE_OUT(-0.8),
    CUBE_OUT(-0.9)
  }
}
