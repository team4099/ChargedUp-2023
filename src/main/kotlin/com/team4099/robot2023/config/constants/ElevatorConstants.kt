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
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ElevatorConstants {

  const val SENSOR_CPR = 42
  val GEAR_RATIO = ((58.0 / 14.0) * (84.0 / 58.0) * (28.0 / 84.0)).gearRatio
  val CARRIAGE_MASS = 10.pounds

  const val FOLLOW_MOTOR_INVERTED = true

  val RAMP_RATE = 0.5.percent.perSecond

  val REAL_KP = 0.85.volts / 1.inches
  val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
  val REAL_KD = 0.0.volts / (1.inches.perSecond)

  val SIM_KP = 1.5.volts / 1.inches
  val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
  val SIM_KD = 0.25.volts / (1.inches.perSecond)

  val ELEVATOR_ANGLE = 49.678.degrees

  val ELEVATOR_GROUND_OFFSET = 6.5.inches

  val SIM_ELEVATOR_KS_SECOND_STAGE = 0.0.volts
  val REAL_ELEVATOR_KS_SECOND_STAGE = 0.54.volts // TODO tune
  val ELEVATOR_KG_SECOND_STAGE = 1.0.volts
  val ELEVATOR_KV_SECOND_STAGE = 0.037.volts / 1.0.inches.perSecond
  val ELEVATOR_KA_SECOND_STAGE = 0.0025.volts / 1.0.inches.perSecond.perSecond

  val SIM_ELEVATOR_KS_FIRST_STAGE = 0.0.volts
  val REAL_ELEVATOR_KS_FIRST_STAGE = 0.54.volts
  val ELEVATOR_KG_FIRST_STAGE = 0.0.volts
  val ELEVATOR_KV_FIRST_STAGE = 0.037.volts / 1.0.inches.perSecond
  val ELEVATOR_KA_FIRST_STAGE = 0.0025.volts / 1.0.inches.perSecond.perSecond

  val VOLTAGE_COMPENSATION = 12.volts
  val PHASE_CURRENT_LIMIT = 80.amps // TODO tune stator current limit

  // TODO figure out what these should be
  val HOMING_POSITION_THRESHOLD = 30.inches
  val HOMING_APPLIED_VOLTAGE = -1.volts
  val HOMING_STALL_CURRENT = 30.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.15.seconds

  // tooth_width * number_teeth = circumference
  // circumference / 2pi = radius
  val SPOOL_RADIUS = 0.005.meters * 32.0 / (2 * PI)

  val MAX_VELOCITY = 75.inches.perSecond // 75
  val MAX_ACCELERATION = 225.inches.perSecond.perSecond // 225

  val ELEVATOR_MAX_EXTENSION = 54.8.inches
  val ELEVATOR_MAX_RETRACTION = 0.0.inches
  val ELEVATOR_SOFT_LIMIT_EXTENSION = 54.5.inches
  val ELEVATOR_SOFT_LIMIT_RETRACTION = 0.5.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION = 45.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION = 5.inches
  val ELEVATOR_IDLE_HEIGHT = 1.0.inches

  val FIRST_STAGE_HEIGHT = 25.05.inches
  val SECOND_STAGE_HEIGHT = 24.inches

  // TODO(do tests to figure out what these values should be)
  val CUBE_DROP_POSITION_DELTA = 3.2.inches
  val CONE_DROP_POSITION_DELTA = 0.0.inches
  val GROUND_INTAKE_CUBE_HEIGHT = 6.0.inches
  val DOUBLE_SUBSTATION_CUBE_OFFSET = 0.0.inches
  val DOUBLE_SUBSTATION_CONE_OFFSET = 11.0.inches
  val SINGLE_SUBSTATION_CUBE_OFFSET = 0.0.inches
  val SINGLE_SUBSTATION_CONE_OFFSET = 24.0.inches
  val SLAM_VELOCITY = 5.0.inches.perSecond

  enum class ElevatorStates(val height: Length) {
    MIN_HEIGHT(ELEVATOR_MAX_RETRACTION),
    GROUND_INTAKE(0.inches),
    LOW_CUBE_SCORE(CUBE_DROP_POSITION_DELTA),
    LOW_CONE_SCORE(CONE_DROP_POSITION_DELTA),
    MID_CUBE_SCORE(midCubeZ + CUBE_DROP_POSITION_DELTA),
    MID_CONE_SCORE(midConeZ + CONE_DROP_POSITION_DELTA),
    CUBE_SHELF_INTAKE(doubleSubstationShelfZ + DOUBLE_SUBSTATION_CUBE_OFFSET),
    CONE_SHELF_INTAKE(doubleSubstationShelfZ + DOUBLE_SUBSTATION_CONE_OFFSET),
    HIGH_CUBE_SCORE(highCubeZ + CUBE_DROP_POSITION_DELTA),
    HIGH_CONE_SCORE(highConeZ + CONE_DROP_POSITION_DELTA),
    MAX_HEIGHT(ELEVATOR_MAX_EXTENSION),
    BETWEEN_TWO_STATES(-Double.NEGATIVE_INFINITY.inches),
    TUNABLE_STATE(30.inches);

    companion object {

      inline fun fromHeightToPosition(height: Length): Length {
        return height / ELEVATOR_ANGLE.sin
      }

      inline fun fromPositionToHeight(position: Length): Length {
        return position * ELEVATOR_ANGLE.sin
      }

      inline fun fromRangeToPosition(range: Length): Length {
        return range / ELEVATOR_ANGLE.cos
      }

      inline fun fromPositionToRange(position: Length): Length {
        return position * ELEVATOR_ANGLE.cos
      }
    }
  }

  val ELEVATOR_TOLERANCE = 0.75.inches
}
