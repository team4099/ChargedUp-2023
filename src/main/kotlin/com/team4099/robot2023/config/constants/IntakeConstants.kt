package com.team4099.robot2023.config.constants

import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object IntakeConstants {

  val NEO_KP = 0.0.volts / 1.degrees
  val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
  val NEO_KD = 0.0.volts / (1.degrees.perSecond)

  val SIM_KP = 0.0.volts / 1.degrees
  val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
  val SIM_KD = 0.0.volts / (1.degrees.perSecond)

  val NEO_ARM_KS = 0.0.volts
  val SIM_ARM_KS = 0.0.volts

  val ARM_KG = 0.0.volts / 1.0.degrees
  val ARM_KV = 0.0.volts / 1.0.degrees.perSecond
  val ARM_KA = 0.0.volts / 1.0.degrees.perSecond.perSecond

  val NEO_ROLLER_KV = 1.0.volts / 1.0.rotations.perMinute
  val SIM_ROLLER_KV = 1.0.volts / 1.0.rotations.perMinute

  val VOLTAGE_COMPENSATION = 12.0.volts

  val ROLLER_CURRENT_LIMIT = 12.amps
  val ARM_CURRENT_LIMIT = 12.amps

  val ROLLER_MOTOR_INVERTED = false
  val ARM_MOTOR_INVERTED = true

  val ROLLER_RAMP_RATE = 0.5.percent.perSecond
  val ARM_RAMP_RATE = 0.5.percent.perSecond

  val ABSOLUTE_ENCODER_OFFSET = 128.degrees // TODO: (Make this fr)
  // From encoder to intake
  val ROLLER_GEAR_RATIO = (36.0 / 18.0)

  // gear reduction from absolute encoder to output
  val ARM_ENCODER_GEAR_RATIO = (32.0 / 16.0)
  // gear reduction from motor to output
  val ARM_OUTPUT_GEAR_RATIO = ((60 / 12) * (80 / 18) * (32.0 / 16.0))

  val MAX_ARM_VELOCITY = 60.degrees.perSecond
  val MAX_ARM_ACCELERATION = 100.degrees.perSecond.perSecond

  val ARM_MAX_ROTATION = 120.degrees
  val ARM_MIN_ROTATION = -1.degrees

  val ENCODER_COUNTS = 42

  val LEFT_MOTOR_INVERTED = false
  val RIGHT_MOTOR_INVERTED = true

  val ARM_MOTOR_CPR = 42

  // TODO(figure out what these should be)

  // Convert in code from an overall angle to a motor angle
  enum class armStates(val position: Angle) {
    DOWN(90.degrees),
    INTAKE(70.degrees),
    RAISED(20.degrees),
    STOWED(0.degrees),
    DUMMY(-Double.NEGATIVE_INFINITY.degrees)
  }

  val ARM_TOLERANCE = 2.degrees

  enum class rollerStates(val velocity: AngularVelocity) {
    INTAKE(600.rotations.perMinute),
    OUTTAKE(300.rotations.perMinute),
    IDLE(0.0.rotations.perMinute),
    NO_SPIN(0.0.rotations.perMinute),
    DUMMY(-Double.NEGATIVE_INFINITY.rotations.perMinute)
  }

  val ROLLERR_TOLERANCE = 10.rotations.perMinute
}
