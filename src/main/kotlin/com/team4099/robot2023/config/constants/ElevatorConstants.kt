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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants {

  const val SENSOR_CPR = 42
  // TODO(Check ratio with design)
  const val GEAR_RATIO = (25.0 / 12.0)
  val CARRIAGE_MASS = 10.pounds
  const val LEFT_MOTOR_INVERTED = true
  const val RIGHT_MOTOR_INVERTED = false
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

  val SPOOL_RADIUS = 1.128.inches

  val MAX_VELOCITY = 50.inches.perSecond
  val MAX_ACCELERATION = 75.inches.perSecond.perSecond

  // TODO(Check height with design)
  val ELEVATOR_MAX_EXTENSION = 52.0.inches
  val ELEVATOR_MAX_RETRACTION = 0.0.inches

  // TODO(do tests to figure out what these values should be)
  val CUBE_DROP_HEIGHT = 0.0.inches
  val CONE_DROP_HEIGHT = 0.0.inches
  val INTAKE_CUBE_SHELF_OFFSET = 0.0.inches
  val INTAKE_CONE_SHELF_OFFSET = 0.0.inches

  enum class DesiredElevatorStates(val height: Length) {
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
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  // assuming that cone drop height is more than the cube drop height
  enum class ActualElevatorStates(val correspondingDesiredState: DesiredElevatorStates) {
    MIN_HEIGHT(DesiredElevatorStates.MIN_HEIGHT),
    GROUND_INTAKE(DesiredElevatorStates.GROUND_INTAKE),
    LOW_CUBE_SCORE(DesiredElevatorStates.LOW_CUBE_SCORE),
    LOW_CONE_SCORE(DesiredElevatorStates.LOW_CONE_SCORE),
    MID_CUBE_SCORE(DesiredElevatorStates.MID_CUBE_SCORE),
    MID_CONE_SCORE(DesiredElevatorStates.MID_CONE_SCORE),
    CUBE_SHELF_INTAKE(DesiredElevatorStates.CUBE_SHELF_INTAKE),
    CONE_SHELF_INTAKE(DesiredElevatorStates.CONE_SHELF_INTAKE),
    HIGH_CUBE_SCORE(DesiredElevatorStates.HIGH_CUBE_SCORE),
    HIGH_CONE_SCORE(DesiredElevatorStates.HIGH_CONE_SCORE),
    MAX_HEIGHT(DesiredElevatorStates.MAX_HEIGHT),
    BETWEEN_TWO_STATES(DesiredElevatorStates.DUMMY);

    companion object {
      fun fromDesiredState(desiredState: DesiredElevatorStates) =
        ActualElevatorStates.values().first { it.correspondingDesiredState == desiredState }
    }
  }

  val ELEVATOR_TOLERANCE = 0.5.inches
}
