package com.team4099.robot2022.config.constants

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.kilo
import com.team4099.lib.units.perSecond

object ElevatorConstants {
  val MAX_HEIGHT_OF_TOP_OF_CARRIAGE = 78.0.inches
  val BOTTOM_HEIGHT_OF_TOP_OF_CARRIAGE = 68.0.inches
  val MIDDLE_HEIGHT_OF_TOP_OF_CARRIAGE = 73.0.inches

  val ELEVATOR_KS = 0.0.volts // TODO tune values
  val ELEVATOR_KV = 0.0.volts / 1.0.meters.perSecond // TODO tune values
  val ELEVATOR_KA = 0.0.volts / 1.0.meters.perSecond.perSecond // TODO tune values
  val ELEVATOR_KG = 0.0.volts // TODO tune values

  val elevatorTolerance: Length = 1.0.inches
  val elevatorMaxExtension: Length = 32.inches
  val elevatorMinExtension: Length = 0.0.inches

  enum class DesiredElevatorStates(val position: Length) {
    BOTTOM_HEIGHT(0.0.inches),
    MID_HEIGHT(16.inches),
    MAX_HEIGHT(32.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  enum class ActualElevatorStates(val correspondingElevatorState: DesiredElevatorStates) {
    BOTTOM(DesiredElevatorStates.BOTTOM_HEIGHT),
    BETWEEN_BOTTOM_AND_MID(DesiredElevatorStates.DUMMY),
    MID_HEIGHT(DesiredElevatorStates.MID_HEIGHT),
    BETWEEN_MID_AND_MAX(DesiredElevatorStates.DUMMY),
    MAX_HEIGHT(DesiredElevatorStates.MAX_HEIGHT)
  }

  val MAX_VELOCITY = 30.inches.perSecond
  val MAX_ACCELERATION = 60.inches.perSecond.perSecond

  val GEARING = 0.0 // TODO get gearing from ryan
  val CARRIAGE_MASS = 0.0.kilo.grams
  val DRUM_RADIUS = 0.0.meters
}
