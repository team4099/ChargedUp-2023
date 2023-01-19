package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
  // Convert in code from an overall angle to a motor angle
  enum class ARM_STATE(val position: Angle){
    DOWN(90.degrees),
    INTAKE(70.degrees),
    STOWED(0.degrees)
  }

  enum class ROLLER_STATE(val power: Double){
    INTAKE(0.8),
    OUTTAKE(-0.8),
    IDLE(0.0)
  }

  val NEO_kP = 0.0.volts/1.degrees
  val NEO_kI = 0.0.volts/(1.degrees * 1.seconds)
  val NEO_kD = 0.0.volts/(1.degrees.perSecond)

  val INTAKE_ZERO = 128 // TODO: (Make this fr)
  // From encoder to intake
  val INTAKE_SENSOR_RATIO = (14.0/32.0) // TODO: (I MADE THIS UP. FIX)
  val INTAKE_ARM_GEAR_RATIO = (24.0/16.0) // TODO: (I MADE THIS UP. FIX)

  val MAX_ARM_VELOCITY = 60.degrees.perSecond
  val MAX_ARM_ACCELERATION = 100.degrees.perSecond.perSecond

  val ENCODER_COUNTS = 8192

  val LEFT_MOTOR_INVERTED = false
  val RIGHT_MOTOR_INVERTED = true

  val VOLTAGE_COMPENSATION = 12.0.volts

  val MOTOR_COUNTS = 42
}
