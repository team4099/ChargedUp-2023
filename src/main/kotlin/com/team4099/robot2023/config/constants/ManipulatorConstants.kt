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
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.milli
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ManipulatorConstants {

  // TODO(Maybe tune feedforward constants)
  val ARM_KS = 0.4.volts
  val ARM_KV = 0.30.volts / 1.0.inches.perSecond
  val ARM_KA = 0.0.volts / 1.0.inches.perSecond.perSecond

  val SIM_ARM_KP = 20.volts / 1.0.inches
  val SIM_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val SIM_ARM_KD = 1.0.volts / 1.0.inches.perSecond

  // TODO(tune these)
  val REAL_ARM_KP = 3.0.volts / 1.0.inches
  val REAL_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val REAL_ARM_KD = 0.0.volts / 1.0.inches.perSecond

  val ARM_RAMP_RATE = 0.5
  val ROLLER_RAMP_RATE = 0.2

  // used to detect intake/outake, values need testing
  // TODO(test this)
  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.3.seconds
  val MANIPULATOR_WAIT_BEFORE_DETECT_VELOCITY_DROP = 0.35.seconds
  val SPIT_OUT_TIME = 0.9.seconds
  val SPIT_OUT_TIME_CONE = 0.6.seconds
  val INTAKE_IN_TIME = 0.5.seconds

  val FILTER_PERIOD = 2.0.milli.seconds

  // TODO(check if these motors are inverted)
  val ARM_MOTOR_INVERTED = true
  val ROLLER_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  val ARM_STATOR_CURRENT_LIMIT = 20.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 40.amps

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
  val CONE_CURRENT_THRESHOLD = 25.amps
  val CONE_ROTATION_THRESHOLD = 40.rotations.perMinute
  val CUBE_CURRENT_THRESHOLD = 30.amps

  val ARM_SPOOL_RADIUS = 0.005.meters * 24.0 / (2 * PI)
  val ARM_MAX_EXTENSION = 8.6.inches
  val ARM_MAX_RETRACTION = 0.inches
  val ARM_TOLERANCE = 0.25.inches
  val ARM_MASS = 10.0.pounds

  // soft limits
  val ARM_SOFTLIMIT_EXTENSION = 8.75.inches
  val ARM_SOFTLIMIT_RETRACTION = 0.25.inches

  val ARM_OPEN_LOOP_SOFTLIMIT_EXTENSION = 8.7.inches
  val ARM_OPEN_LOOP_SOFTLIMIT_RETRACTION = 0.5.inches

  // TODO(check for accuracy)
  val ARM_MAX_VELOCITY = 40.inches.perSecond
  val ARM_MAX_ACCELERATION = 400.inches.perSecond.perSecond

  val MOMENT_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared

  // Tolerance for determining currentRollerState
  val ROLLER_VOLTAGE_TOLERANCE = 0.4.volts

  val IDLE_VOLTAGE = 0.0.volts
  val CONE_IDLE = 7.volts
  val CUBE_IDLE = -3.volts
  val CONE_IN = 12.volts
  val CUBE_IN = -6.volts
  val CONE_OUT = -12.volts
  val CUBE_OUT = 12.volts

  // TODO(test voltage values)
  enum class RollerStates(val voltage: ElectricalPotential) {
    NO_SPIN(0.volts),
    CONE_IDLE(3.5.volts),
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

  val ENABLE_ROLLER = 1.0
  val ENABLE_EXTENSION = 1.0

  val MIN_EXTENSION = 1.0.inches
  val SINGLE_SUBSTATION_INTAKE_EXTENSION = 1.0.inches
  val DOUBLE_SUBSTATION_SHELF_INTAKE_EXTENSION = 7.0.inches
  val LOW_CUBE_SCORE_EXTENSION = 1.3.inches
  val MID_CUBE_SCORE_EXTENSION = 3.0.inches
  val HIGH_CUBE_SCORE_EXTENSION = 8.6.inches
  val LOW_CONE_SCORE_EXTENSION = 7.5.inches
  val MID_CONE_SCORE_EXTENSION = 5.0.inches
  val HIGH_CONE_SCORE_EXTENSION = 7.8.inches
  val INTAKE_CUBE_FROM_GROUND_EXTENSION = 1.0.inches
  val INTAKE_CONE_FROM_GROUND_EXTENSION = 7.0.inches
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
