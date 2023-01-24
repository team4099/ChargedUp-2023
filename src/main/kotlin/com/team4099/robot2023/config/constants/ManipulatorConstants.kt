package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
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
  val ROLLER_RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  val INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.9.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  val ARM_MOTOR_INVERTED = false
  val ROLLER_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  val ARM_STATOR_CURRENT_LIMIT = 25.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 25.amps
  const val SENSOR_CPR = 42.0

  // waiting for deisgn
  const val ARM_GEAR_RATIO = 1.0
  const val ROLLER_GEAR_RATIO = 1.0

  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps

  val ARM_SPOOL_RADIUS = 1.0.inches

  // TODO(What is the inertial value)
  val MOMENT_INERTIA = 0.0045

  val ROLLER_POWER_TOLERANCE = 0.1.volts

  enum class RollerStates(val voltage: ElectricalPotential) {
    // TODO: 1/9/23
    // figure out if idle should be 0.1 or -0.1 or smth
    IDLE(0.0.volts),
    CONE_IN(10.volts),
    CUBE_IN(12.volts),
    CONE_OUT(-10.volts),
    CUBE_OUT(-12.volts),
    DUMMY(-Double.NEGATIVE_INFINITY.volts);

    companion object {
      fun fromRollerState(state: RollerStates) {
        RollerStates.values().first { it.voltage == state.voltage }
      }
    }
  }

  enum class DesiredArmStates(val position: Length) {
    MIN_EXTENSION(0.inches),
    SHELF_INTAKE_EXTENSION(4.inches),
    HIGH_SCORE_EXTENSION(8.inches),
    MAX_EXTENSION(10.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  enum class ActualArmStates(val correspondingDesiredState: DesiredArmStates) {
    MIN_EXTENSION(DesiredArmStates.MIN_EXTENSION),
    SHELF_INTAKE_EXTENSION(DesiredArmStates.SHELF_INTAKE_EXTENSION),
    HIGH_SCORE_EXTENSION(DesiredArmStates.HIGH_SCORE_EXTENSION),
    MAX_EXTENSION(DesiredArmStates.MAX_EXTENSION),
    BETWEEN_TWO_STATES(DesiredArmStates.DUMMY);

    companion object {
      fun fromDesiredState(desiredState: DesiredArmStates) =
        ActualArmStates.values().first { it.correspondingDesiredState == desiredState }
    }
  }
}
