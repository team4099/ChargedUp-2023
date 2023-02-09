package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.driven
import org.team4099.lib.units.derived.driving
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.reduction
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object GroundIntakeConstants {

  object PID {
    val NEO_KP = 0.0.volts / 1.degrees
    val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
    val NEO_KD = 0.0.volts / (1.degrees.perSecond)

    val SIM_KP = 1.5.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.01.volts / (1.degrees.perSecond)

    val ARM_KS = 0.0.volts

    val ARM_KG = 1.582887.volts
    val ARM_KV = 0.87.volts / 1.0.radians.perSecond
    val ARM_KA = 0.04.volts / 1.0.radians.perSecond.perSecond
  }

  val VOLTAGE_COMPENSATION = 12.0.volts

  val ROLLER_CURRENT_LIMIT = 30.amps // TODO TUNE
  val ARM_CURRENT_LIMIT = 30.amps // TODO TUNE

  const val ROLLER_MOTOR_INVERTED = false
  const val ARM_MOTOR_INVERTED = false

  val ROLLER_RAMP_RATE = 50.percent.perSecond

  val ABSOLUTE_ENCODER_OFFSET = 128.degrees // TODO: (Make this fr)
  // From encoder to intake
  val ROLLER_GEAR_RATIO = (36.0.driven / 18.0.driving).reduction

  // units are kg * m^2
  val ROLLER_MOMENT_INERTIA = 0.00313.kilo.grams * 1.0.meters.squared

  // gear reduction from absolute encoder to output
  val ARM_ENCODER_GEAR_RATIO = (32.0.driven / 16.0.driving).reduction
  // gear reduction from motor to output
  val ARM_OUTPUT_GEAR_RATIO =
    ((60.0.driven / 12.0.driving) * (80.0.driven / 18.0.driving) * (32.0.driven / 16.0.driving))
      .reduction

  val ARM_LENGTH = 15.0.inches // TODO figure out via cad (this should be distance to com)

  val ARM_MASS = 9.pounds

  // units are kg * m^2
  val ARM_MOMENT_INERTIA = ARM_MASS * ARM_LENGTH.squared * 1 / 3

  val MAX_ARM_VELOCITY =
    360.degrees
      .perSecond // todo mess with velocity and arm to get arm movements to sub 0.25 seconds
  // (ish)
  val MAX_ARM_ACCELERATION = 400.degrees.perSecond.perSecond

  val ARM_MAX_ROTATION = 56.6.degrees
  val ARM_MIN_ROTATION = 0.degrees

  val ARM_OPEN_LOOP_MAX_ROTATION = 45.5.degrees
  val ARM_OPEN_LOOP_MIN_ROTATION = 10.0.degrees

  val ARM_TOLERANCE = 2.degrees
  val VOLTAGE_TOLERANCE = 0.3.volts // such a terrible constant to have but whatever

  val INTAKE_ANGLE = 4.4.degrees
  val OUTTAKE_ANGLE = 4.4.degrees
  val STOWED_UP_ANGLE = 56.6.degrees
  val INTAKE_VOLTAGE = 3.0.volts
  val OUTTAKE_VOLTAGE = (-3.0).volts
  val STOWED_DOWN_ANGLE = 4.4.degrees
  val NEUTRAL_VOLTAGE = 0.0.volts

  enum class ArmStates {
    OPEN_LOOP,
    TARGETING_POSITION,
    UNINITIALIZED
  }

  enum class RollerStates(val voltage: ElectricalPotential) {
    INTAKE(3.0.volts),
    OUTTAKE((-3.0).volts),
    IDLE(0.0.volts),
    NO_SPIN(0.0.volts),
    DUMMY(-Double.NEGATIVE_INFINITY.volts);

    companion object {
      fun fromVoltageToRollerState(voltage: ElectricalPotential): RollerStates {
        return values().firstOrNull { (it.voltage - voltage).absoluteValue <= VOLTAGE_TOLERANCE }
          ?: DUMMY
      }
    }
  }
}
