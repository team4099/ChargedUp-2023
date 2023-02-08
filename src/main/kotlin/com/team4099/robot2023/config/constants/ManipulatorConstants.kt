package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.reduction
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorConstants {

  // TODO(Maybe tune feedforward constants)
  val ARM_KS = 0.002.volts
  val ARM_KV = 0.2.volts / 1.0.inches.perSecond
  val ARM_KA = 0.0.volts / 1.0.inches.perSecond.perSecond

  val SIM_ARM_KP = 20.volts / 1.0.inches
  val SIM_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val SIM_ARM_KD = 1.0.volts / 1.0.inches.perSecond

  // TODO(tune these)
  val REAL_ARM_KP = 1.0.volts / 1.0.inches
  val REAL_ARM_KI = 0.0.volts / (1.0.inches * 1.0.seconds)
  val REAL_ARM_KD = 0.25.volts / 1.0.inches.perSecond

  val ARM_RAMP_RATE = 0.5
  val ROLLER_RAMP_RATE = 0.5

  // used to detect intake/outake, values need testing
  // TODO(test this)
  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.75.seconds
  val WAIT_FOR_STATE_TO_CHANGE = 0.8.seconds

  // TODO(check if these motors are inverted)
  val ARM_MOTOR_INVERTED = false
  val ROLLER_MOTOR_INVERTED = false

  // TODO(figure out what current limit should be)
  val ARM_STATOR_CURRENT_LIMIT = 25.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 25.amps

  val ARM_VOLTAGE_COMPENSATION = 12.volts
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts

  val ARM_HOMING_POSITION_THESHOLD = 5.inches
  val ARM_HOMING_APPLIED_VOLTAGE = -0.5.volts
  val ARM_HOMING_STALL_CURRENT = 15.amps

  const val SENSOR_CPR = 42.0

  val ARM_GEAR_RATIO = 11.25.reduction
  val ROLLER_GEAR_RATIO = 18.0.reduction

  // TODO: Change current thresholds
  val CONE_CURRENT_THRESHOLD = 15.amps
  val CUBE_CURRENT_THRESHOLD = 15.amps

  val ARM_SPOOL_RADIUS = 0.581.inches
  val ARM_MAX_EXTENSION = 20.inches
  val ARM_MAX_RETRACTION = 0.inches
  val ARM_TOLERANCE = 0.25.inches
  val ARM_MASS = 10.0.pounds

  // soft limits
  val ARM_SOFTLIMIT_EXTENSION = 10.75.inches
  val ARM_SOFTLIMIT_RETRACTION = 0.25.inches

  val ARM_OPEN_LOOP_SOFTLIMIT_EXTENSION = 8.inches
  val ARM_OPEN_LOOP_SOFTLIMIT_RETRACTION = 0.5.inches

  // TODO(check for accuracy)
  val ARM_MAX_VELOCITY = 30.inches.perSecond
  val ARM_MAX_ACCELERATION = 15.inches.perSecond.perSecond

  val MOMENT_INERTIA = 0.0000478

  // Tolerance for determining currentRollerState
  val ROLLER_VOLTAGE_TOLERANCE = 0.4.volts

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
