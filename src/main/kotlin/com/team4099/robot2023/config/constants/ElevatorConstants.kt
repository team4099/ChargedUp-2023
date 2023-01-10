package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches

object ElevatorConstants {
  val ELEVATOR_MAX_EXTENSION = 50.0.inches
  val ELEVATOR_MAX_RETRACTION = 0.0.inches

  enum class DesiredElevatorStates(val position: Length) {
    MIN_HEIGHT(0.0.inches),
    // TODO(Check height with design)
    GROUND_INTAKE_HEIGHT(1.inches),
    // TODO(Check height with design)
    LOW_SCORE_HEIGHT(1.5.inches),
    // TODO(Check height with design)
    MID_SCORE_HEIGHT(10.inches),
    // TODO(Check height with design)
    SHELF_INTAKE_HEIGHT(15.inches),
    // TODO(Check height with design)
    HIGH_SCORE_HEIGHT(30.inches),
    // TODO(Check height with design)
    MAX_HEIGHT(50.0.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  enum class ActualElevatorStates(val correspondingDesiredState: DesiredElevatorStates) {
    MIN_HEIGHT(DesiredElevatorStates.MIN_HEIGHT),
    BETWEEN_MIN_AND_GROUND_INTAKE(DesiredElevatorStates.DUMMY),
    GROUND_INTAKE(DesiredElevatorStates.GROUND_INTAKE_HEIGHT),
    BETWEEN_GROUND_INTAKE_AND_LOW(DesiredElevatorStates.DUMMY),
    LOW_SCORE_HEIGHT(DesiredElevatorStates.LOW_SCORE_HEIGHT),
    BETWEEN_LOW_AND_MID(DesiredElevatorStates.DUMMY),
    MID_SCORE_HEIGHT(DesiredElevatorStates.MID_SCORE_HEIGHT),
    BETWEEN_MID_AND_SHELF_INTAKE(DesiredElevatorStates.DUMMY),
    SHELF_INTAKE_HEIGHT(DesiredElevatorStates.SHELF_INTAKE_HEIGHT),
    BETWEEN_SHELF_INTAKE_AND_HIGH(DesiredElevatorStates.DUMMY),
    HIGH_SCORE_HEIGHT(DesiredElevatorStates.HIGH_SCORE_HEIGHT),
    BETWEEN_HIGH_AND_MAX(DesiredElevatorStates.DUMMY),
    MAX_HEIGHT(DesiredElevatorStates.MAX_HEIGHT)
  }
}
