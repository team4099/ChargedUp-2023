package com.team4099.robot2023.config.constants

import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.reduction
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ManipulatorConstants {

  // PID constants
  val ARM_KS = 0.002.volts
  val ARM_KV = 0.2.volts / 1.0.inches.perSecond
  val ARM_KA = 0.0.volts / 1.0.inches.perSecond.perSecond

  val SIM_ARM_KP = 10.volts / 1.0.inches
  val SIM_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val SIM_ARM_KD = 0.0.volts / 1.0.inches.perSecond

  val REAL_ARM_KP = 1.0.volts / 1.0.inches
  val REAL_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val REAL_ARM_KD = 0.25.volts / 1.0.inches.perSecond

  //Constant for rpm to voltage
  val SIM_ROLLER_KV = 0.0039.volts / 1.0.rotations.perMinute
  val REAL_ROLLER_KV = 0.0039.volts / 1.0.rotations.perMinute

  val ARM_RAMP_RATE = 0.5
  val ROLLER_RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  // TODO(check if these motors are inverted)
  val ARM_MOTOR_INVERTED = false
  val ROLLER_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  val ARM_STATOR_CURRENT_LIMIT = 25.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 25.amps

  const val SENSOR_CPR = 42.0

  val ARM_GEAR_RATIO = 11.25.reduction
  val ROLLER_GEAR_RATIO = 18.0.reduction

  val CONE_CURRENT_THRESHOLD = 15.amps
  val CUBE_CURRENT_THRESHOLD = 15.amps

  val ARM_SPOOL_RADIUS = 0.581.inches
  val ARM_MAX_EXTENSION = 11.inches
  val ARM_MAX_RETRACTION = 0.inches
  val ARM_TOLERANCE = 0.25.inches
  val ARM_MASS = 10.0.pounds

  // TODO(check for accuracy)
  val ARM_MAX_VELOCITY = 30.inches.perSecond
  val ARM_MAX_ACCELERATION = 15.inches.perSecond.perSecond

  val MOMENT_INERTIA = 0.0000478

  //tolerance for determining currentRollerState
  val ROLLER_SPEED_TOLERANCE = 10.rotations.perMinute

  enum class RollerStates(val velocity: AngularVelocity) {
    NO_SPIN(0.rotations.perMinute),
    CONE_IDLE(60.rotations.perMinute),
    CUBE_IDLE(-30.rotations.perMinute),
    CONE_IN(300.rotations.perMinute),
    CUBE_IN(-240.rotations.perMinute),
    CONE_OUT(-300.rotations.perMinute),
    CUBE_OUT(240.rotations.perMinute),
    DUMMY(-Double.NEGATIVE_INFINITY.rotations.perMinute)
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
      //Converts desired state enum to actual state enum
      fun fromDesiredState(desiredState: DesiredArmStates) =
        ActualArmStates.values().first { it.correspondingDesiredState == desiredState }
    }
  }
}
