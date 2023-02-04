package com.team4099.robot2023.config.constants

import com.team4099.robot2023.config.constants.FieldConstants.Grids.highConeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.highCubeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.midConeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.midCubeZ
import com.team4099.robot2023.config.constants.FieldConstants.LoadingZone.doubleSubstationShelfZ
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ElevatorConstants {

  const val SENSOR_CPR = 42
  const val GEAR_RATIO = (25.0 / 12.0)
  val CARRIAGE_MASS = 10.pounds

  // TODO(check inversion)
  const val LEFT_MOTOR_INVERTED = true
  const val RIGHT_MOTOR_INVERTED = !LEFT_MOTOR_INVERTED

  val RAMP_RATE = 0.5.percent.perSecond

  val REAL_KP = 1.0.volts / 1.meters
  val REAL_KI = 0.0.volts / (1.meters * 1.seconds)
  val REAL_KD = 0.0.volts / (1.meters.perSecond)

  val SIM_KP = 1.5.volts / 1.inches
  val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
  val SIM_KD = 0.25.volts / (1.inches.perSecond)

  val ELEVATOR_ANGLE = 51.68.degrees

  val SIM_ELEVATOR_KS = 0.0.volts
  val REAL_ELEVATOR_KS = 0.0.volts // TODO tune
  val ELEVATOR_KG = 1.1066.volts
  val ELEVATOR_KV = 1.59.volts / 1.0.meters.perSecond
  val ELEVATOR_KA = 0.1.volts / 1.0.meters.perSecond.perSecond

  val VOLTAGE_COMPENSATION = 12.volts
  val PHASE_CURRENT_LIMIT = 12.amps // TODO tune stator current limit

  // TODO figure out what these should be
  val HOMING_POSITION_THRESHOLD = 30.inches
  val HOMING_APPLIED_VOLTAGE = -0.5.volts
  val HOMING_STALL_CURRENT = 15.amps

  // tooth_width * number_teeth = circumference
  // circumference / 2pi = radius
  val SPOOL_RADIUS = 0.005.meters * 32.0 / (2 * PI)

  val MAX_VELOCITY = 50.inches.perSecond
  val MAX_ACCELERATION = 75.inches.perSecond.perSecond

  val ELEVATOR_MAX_EXTENSION = 100.0.inches
  val ELEVATOR_MAX_RETRACTION = 0.0.inches
  val ELEVATOR_SOFTLIMIT_EXTENSION = 51.5.inches
  val ELEVATOR_SOFTLIMIT_RETRACTION = 0.5.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION = 45.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION = 5.inches

  // TODO(do tests to figure out what these values should be)
  val CUBE_DROP_HEIGHT = 0.0.inches
  val CONE_DROP_HEIGHT = 0.0.inches
  val INTAKE_CUBE_SHELF_OFFSET = 0.0.inches
  val INTAKE_CONE_SHELF_OFFSET = 0.0.inches

  enum class ElevatorStates(val height: Length) {
    MIN_HEIGHT(ELEVATOR_MAX_RETRACTION),
    GROUND_INTAKE(0.inches),
    LOW_CUBE_SCORE(CUBE_DROP_HEIGHT),
    LOW_CONE_SCORE(CONE_DROP_HEIGHT),
    MID_CUBE_SCORE(midCubeZ + CUBE_DROP_HEIGHT),
    MID_CONE_SCORE(midConeZ + CONE_DROP_HEIGHT),
    CUBE_SHELF_INTAKE(doubleSubstationShelfZ + INTAKE_CUBE_SHELF_OFFSET),
    CONE_SHELF_INTAKE(doubleSubstationShelfZ + INTAKE_CONE_SHELF_OFFSET),
    HIGH_CUBE_SCORE(highCubeZ + CUBE_DROP_HEIGHT),
    HIGH_CONE_SCORE(highConeZ + CONE_DROP_HEIGHT),
    MAX_HEIGHT(ELEVATOR_MAX_EXTENSION),
    BETWEEN_TWO_STATES(-Double.NEGATIVE_INFINITY.inches);

    companion object {

      fun fromHeightToPosition(height: Length): Length {
        return height / ELEVATOR_ANGLE.sin
      }

      fun fromPositionToHeight(position: Length): Length {
        return position * ELEVATOR_ANGLE.sin
      }
    }
  }

  val ELEVATOR_TOLERANCE = 0.5.inches
}
