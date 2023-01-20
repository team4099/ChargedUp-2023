package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorConstants {
  // TODO: 1/9/23

  // PID constants
  val ARM_KS = 1.0.volts
  val ARM_KV = 1.0.volts / 1.0.meters.perSecond
  val ARM_KA = 1.0.volts / 1.0.meters.perSecond.perSecond

  // make sure constants are their actual values
  val ARM_RAMP_RATE = 0.5
  val INTAKE_RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  val INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.9.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  val ARM_MOTOR_INVERTED = false
  val INTAKE_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  const val ARM_SUPPLY_CURRENT_LIMIT = 25
  const val INTAKE_SUPPLY_CURRENT_LIMIT = 25
  const val SENSOR_CPR = 42.0
  // waiting for deisgn
  const val ARM_GEAR_RATIO = 1.0
  const val INTAKE_GEAR_RATIO = 1.0

  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps

  // TODO(figure out what voltage compensation should be)
  val ARM_VOLTAGE_COMPENSATION = 10.0.volts
  val INTAKE_VOLTAGE_COMPENSATION = 10.0.volts

  val ARM_SPOOL_RADIUS = 1.0.inches
  // TODO(What is the inertial value)
  val MOMENT_INERTIA = 0.0045

  enum class RollerState(val speed: Double) {
    // TODO: 1/9/23
    // figure out if idle should be 0.1 or -0.1 or smth
    IDLE(0.0),
    CONE_IN(0.8),
    CUBE_IN(0.9),
    CONE_OUT(-0.8),
    CUBE_OUT(-0.9)
  }
}
