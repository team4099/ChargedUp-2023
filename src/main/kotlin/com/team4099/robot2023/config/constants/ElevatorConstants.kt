package com.team4099.robot2023.config.constants

import com.team4099.robot2023.config.constants.FieldConstants.Grids.highConeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.highCubeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.midConeZ
import com.team4099.robot2023.config.constants.FieldConstants.Grids.midCubeZ
import com.team4099.robot2023.config.constants.FieldConstants.LoadingZone.doubleSubstationShelfZ
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees

object ElevatorConstants {
  val ELEVATOR_MAX_EXTENSION = 50.0.inches
  val ELEVATOR_MAX_RETRACTION = 0.0.inches
  // TODO(Check height with design)
  val ELEVATOR_ANGLE = 45.degrees
  // TODO(Check height with design)
  val CUBE_DROP_HEIGHT = 0.0.inches
  val CONE_DROP_HEIGHT = 0.0.inches
  val INTAKE_CUBE_SHELF_OFFSET = 0.0.inches
  val INTAKE_CONE_SHELF_OFFSET = 0.0.inches

  enum class DesiredElevatorStates(val height: Length) {
    MIN_HEIGHT(0.0.inches),
    GROUND_INTAKE(1.inches),
    LOW_CUBE_SCORE(CUBE_DROP_HEIGHT),
    LOW_CONE_SCORE(CONE_DROP_HEIGHT),
    MID_CUBE_SCORE(midCubeZ + CUBE_DROP_HEIGHT),
    MID_CONE_SCORE(midConeZ + CONE_DROP_HEIGHT),
    CUBE_SHELF_INTAKE(doubleSubstationShelfZ + INTAKE_CUBE_SHELF_OFFSET),
    CONE_SHELF_INTAKE(doubleSubstationShelfZ + INTAKE_CONE_SHELF_OFFSET),
    HIGH_CUBE_SCORE(highCubeZ + CUBE_DROP_HEIGHT),
    HIGH_CONE_SCORE(highConeZ + CONE_DROP_HEIGHT),
    MAX_HEIGHT(50.0.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  // assuming that cone drop height is more than the cube drop height
  enum class ActualElevatorStates(val correspondingDesiredState: DesiredElevatorStates) {
    MIN_HEIGHT(DesiredElevatorStates.MIN_HEIGHT),
    BETWEEN_MIN_AND_GROUND_INTAKE(DesiredElevatorStates.DUMMY),
    GROUND_INTAKE(DesiredElevatorStates.GROUND_INTAKE),
    BETWEEN_GROUND_INTAKE_AND_LOW_CUBE(DesiredElevatorStates.DUMMY),
    LOW_CUBE_SCORE(DesiredElevatorStates.LOW_CUBE_SCORE),
    BETWEEN_LOW_CUBE_AND_LOW_CONE(DesiredElevatorStates.DUMMY),
    LOW_CONE_SCORE(DesiredElevatorStates.LOW_CONE_SCORE),
    BETWEEN_LOW_CONE_AND_MID_CUBE(DesiredElevatorStates.DUMMY),
    MID_CUBE_SCORE(DesiredElevatorStates.MID_CUBE_SCORE),
    BETWEEN_MID_CUBE_AND_MID_CONE(DesiredElevatorStates.DUMMY),
    MID_CONE_SCORE(DesiredElevatorStates.MID_CONE_SCORE),
    BETWEEN_MID_CONE_AND_CUBE_SHELF_INTAKE(DesiredElevatorStates.DUMMY),
    CUBE_SHELF_INTAKE(DesiredElevatorStates.CUBE_SHELF_INTAKE),
    BETWEEN_CUBE_SHELF_INTAKE_AND_CONE_SHELF_INTAKE(DesiredElevatorStates.DUMMY),
    CONE_SHELF_INTAKE(DesiredElevatorStates.CONE_SHELF_INTAKE),
    BETWEEN_CONE_SHELF_INTAKE_AND_HIGH_CUBE(DesiredElevatorStates.DUMMY),
    HIGH_CUBE_SCORE(DesiredElevatorStates.HIGH_CUBE_SCORE),
    BETWEEN_HIGH_CUBE_AND_HIGH_CONE(DesiredElevatorStates.DUMMY),
    HIGH_CONE_SCORE(DesiredElevatorStates.HIGH_CONE_SCORE),
    BETWEEN_HIGH_CONE_AND_MAX_HEIGHT(DesiredElevatorStates.DUMMY),
    MAX_HEIGHT(DesiredElevatorStates.MAX_HEIGHT),
  }
}
