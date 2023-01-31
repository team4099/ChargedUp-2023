package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
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

  // Constant for rpm to voltage
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

  // TODO: Change current thresholds
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

  // Tolerance for determining currentRollerState
  val ROLLER_VOLTAGE_TOLERANCE = 0.4.volts

  enum class RollerStates(val voltage: ElectricalPotential) {
    NO_SPIN(0.volts),
    CONE_IDLE(2.4.volts),
    CUBE_IDLE(-1.2.volts),
    CONE_IN(12.volts),
    CUBE_IN(-9.6.volts),
    CONE_OUT(-12.volts),
    CUBE_OUT(9.6.volts),
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

  enum class DesiredArmStates(val position: Length) {
    MIN_EXTENSION(0.inches),
    SHELF_INTAKE_EXTENSION(4.inches),
    HIGH_SCORE_EXTENSION(8.inches),
    MAX_EXTENSION(10.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches);

    companion object {
      /**
       * Uses the arm position to find the best fit for the current arm state which is then passed
       * into the ActualArmState to calculate the proper state.
       *
       * @param armPosition The current position of the arm
       */
      fun fromArmPositionToState(armPosition: Length): ActualArmStates {
        return ActualArmStates.fromDesiredState(
          values().firstOrNull { (it.position - armPosition).absoluteValue <= ARM_TOLERANCE }
            ?: DUMMY
        )
      }
    }
  }

  enum class ActualArmStates(val correspondingDesiredState: DesiredArmStates) {
    MIN_EXTENSION(DesiredArmStates.MIN_EXTENSION),
    SHELF_INTAKE_EXTENSION(DesiredArmStates.SHELF_INTAKE_EXTENSION),
    HIGH_SCORE_EXTENSION(DesiredArmStates.HIGH_SCORE_EXTENSION),
    MAX_EXTENSION(DesiredArmStates.MAX_EXTENSION),
    BETWEEN_TWO_STATES(DesiredArmStates.DUMMY);

    companion object {
      // Converts desired state enum to actual state enum
      fun fromDesiredState(desiredState: DesiredArmStates) =
        ActualArmStates.values().first { it.correspondingDesiredState == desiredState }
    }
  }
}
