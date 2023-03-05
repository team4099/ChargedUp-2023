package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.driven
import org.team4099.lib.units.derived.driving
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ManipulatorConstants {

  // TODO(Maybe tune feedforward constants)
  val ARM_KS = 0.4.volts
  val ARM_KV = 0.16.volts / 1.0.inches.perSecond
  val ARM_KA = 0.0.volts / 1.0.inches.perSecond.perSecond

  val SIM_ARM_KP = 20.volts / 1.0.inches
  val SIM_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val SIM_ARM_KD = 1.0.volts / 1.0.inches.perSecond

  // TODO(tune these)
  val REAL_ARM_KP = 2.0.volts / 1.0.inches
  val REAL_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val REAL_ARM_KD = 0.0.volts / 1.0.inches.perSecond

  val ARM_RAMP_RATE = 0.5
  val ROLLER_RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  // TODO(test this)
  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val SPIT_OUT_TIME = 0.8.seconds
  val INTAKE_IN_TIME = 0.5.seconds

  // TODO(check if these motors are inverted)
  val ARM_MOTOR_INVERTED = true
  val ROLLER_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  val ARM_STATOR_CURRENT_LIMIT = 20.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 20.amps

  val ARM_VOLTAGE_COMPENSATION = 12.volts
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts

  val ARM_HOMING_POSITION_THESHOLD = 5.inches
  val ARM_HOMING_APPLIED_VOLTAGE = -1.volts
  val ARM_HOMING_STALL_CURRENT = 10.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.15.seconds

  const val SENSOR_CPR = 42.0

  val ARM_GEAR_RATIO = ((84.0.driven / 29.0.driving) * (60.0.driven / 16.0.driving)).gearRatio
  val ROLLER_GEAR_RATIO = 20.0.gearRatio

  // TODO: Change current thresholds
  val CONE_CURRENT_THRESHOLD = 18.amps
  val CUBE_CURRENT_THRESHOLD = 23.amps

  val ARM_SPOOL_RADIUS = 0.005.meters * 24.0 / (2 * PI)
  val ARM_MAX_EXTENSION = 8.inches
  val ARM_MAX_RETRACTION = 0.inches
  val ARM_TOLERANCE = 0.25.inches
  val ARM_MASS = 10.0.pounds

  // soft limits
  val ARM_SOFTLIMIT_EXTENSION = 7.95.inches
  val ARM_SOFTLIMIT_RETRACTION = 0.25.inches

  val ARM_OPEN_LOOP_SOFTLIMIT_EXTENSION = 8.inches
  val ARM_OPEN_LOOP_SOFTLIMIT_RETRACTION = 0.5.inches

  // TODO(check for accuracy)
  val ARM_MAX_VELOCITY = 40.inches.perSecond
  val ARM_MAX_ACCELERATION = 400.inches.perSecond.perSecond

  val MOMENT_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared

  // Tolerance for determining currentRollerState
  val ROLLER_VOLTAGE_TOLERANCE = 0.4.volts

  val IDLE_VOLTAGE = 0.0.volts
  val CONE_IDLE = 0.2.volts
  val CUBE_IDLE = -3.volts
  val CONE_IN = 7.5.volts
  val CUBE_IN = -6.volts
  val CONE_OUT = -6.volts
  val CUBE_OUT = 6.volts

  // TODO(test voltage values)
  enum class RollerStates(val voltage: ElectricalPotential) {
    NO_SPIN(0.volts),
    CONE_IDLE(2.4.volts),
    CUBE_IDLE(-1.2.volts),
    CONE_IN(12.volts),
    CUBE_IN(-9.6.volts),
    CONE_OUT(-12.volts),
    CUBE_OUT(9.6.volts),
    TUNABLE_STATE(4.0.volts),
    DUMMY(-Double.NEGATIVE_INFINITY.volts);

    companion object {
      /**
       * Uses roller velocity to calculate the current roller state via the difference between the
       * current roller velocity and the velocity of the state to find the best fit.
       *
       * @param rollerVelocity The velocity of the roller as represented in radians per second.
       */
      fun fromRollerVoltageToState(rollerVoltage: ElectricalPotential): RollerStates {
        return values().firstOrNull {
          (it.voltage - rollerVoltage).absoluteValue <= ROLLER_VOLTAGE_TOLERANCE
        }
          ?: DUMMY
      }
    }
  }

  val MIN_EXTENSION = 1.0.inches
  val SINGLE_SUBSTATION_INTAKE_EXTENSION = 4.0.inches
  val DOUBLE_SUBSTATION_SHELF_INTAKE_EXTENSION = 7.0.inches
  val LOW_SCORE_EXTENSION = 1.0.inches
  val MID_SCORE_EXTENSION = 7.0.inches
  val HIGH_SCORE_EXTENSION = 7.5.inches
  val INTAKE_CUBE_FROM_GROUND_EXTENSION = 1.0.inches
  val INTAKE_CONE_FROM_GROUND_EXTENSION = 4.5.inches
  val MAX_EXTENSION = 7.8.inches

  // TODO(figure out the values)
  enum class ArmStates(val position: Length) {
    MIN_EXTENSION(0.inches),
    SHELF_INTAKE_EXTENSION(4.inches),
    HIGH_SCORE_EXTENSION(8.inches),
    MAX_EXTENSION(10.inches),
    TUNABLE_STATE(5.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches);

    companion object {
      /**
       * Uses the arm position to find the best fit for the current arm state which is then passed
       * into the ActualArmState to calculate the proper state.
       *
       * @param armPosition The current position of the arm
       */
      fun fromArmPositionToState(armPosition: Length): ArmStates {
        return values().firstOrNull { (it.position - armPosition).absoluteValue <= ARM_TOLERANCE }
          ?: DUMMY
      }
    }
  }
}
