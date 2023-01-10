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
    SHELF_INTAKE_HEIGHT(15.inches),
    // TODO(Check height with design)
    LOW_SCORE_HEIGHT(1.5.inches),
    // TODO(Check height with design)
    MID_SCORE_HEIGHT(20.inches),
    // TODO(Check height with design)
    HIGH_SCORE_HEIGHT(30.inches),
    // TODO(Check height with design)
    MAX_HEIGHT(50.0.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  enum class ActualElevatorStates(val correspondingDesiredState: DesiredElevatorStates) {
    START(DesiredElevatorStates.MIN_HEIGHT),
    BETWEEN_MIN_AND_GROUND_INTAKE(DesiredElevatorStates.DUMMY),
    GROUND_INTAKE(DesiredElevatorStates.GROUND_INTAKE_HEIGHT)
  }
}
