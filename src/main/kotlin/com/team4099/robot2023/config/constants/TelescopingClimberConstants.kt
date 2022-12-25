package com.team4099.robot2023.config.constants

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.perSecond

object TelescopingClimberConstants {
  const val SENSOR_CPR = 2048
  const val GEAR_RATIO = (8.0 / 54.0) * (28.0 / 44.0)

  const val KP = 0.15
  const val KI = 0.0
  const val KD = 0.0
  const val KFF = 0.0

  val NO_LOAD_KS = 1.824.volts
  val NO_LOAD_KG = (-1.824).volts
  val NO_LOAD_KV = 13.762.volts / 1.0.meters.perSecond
  val NO_LOAD_KA = 0.64434.volts / 1.0.meters.perSecond.perSecond

  val LOAD_KS = 0.0.volts
  val LOAD_KG = (-0.88).volts
  val LOAD_KV = 2.83.volts / 1.0.meters.perSecond
  val LOAD_KA = 0.11.volts / 1.0.meters.perSecond.perSecond

  val SPOOL_RADIUS = 0.5.inches
  val LEFT_SPOOL_RADIUS = 0.5125.inches
  val RIGHT_SPOOL_RADIUS = 0.515.inches
  val MAX_VELOCITY = 35.inches.perSecond
  val MAX_ACCELERATION = 60.inches.perSecond.perSecond

  const val BOTTOM_SAFETY_THRESHOLD = 0
  const val TOP_SAFETY_THRESHOLD = 0

  enum class DesiredTelescopeStates(val position: Length) {
    START(0.0.inches),
    MAX_RETRACT(1.inches),
    MAX_EXTENSION(25.5.inches),
    DUMMY(-Double.NEGATIVE_INFINITY.inches)
  }

  enum class ActualTelescopeStates(val correspondingDesiredState: DesiredTelescopeStates) {
    START(DesiredTelescopeStates.START),
    BETWEEN_START_AND_MAX_RETRACT(DesiredTelescopeStates.DUMMY),
    MAX_RETRACT(DesiredTelescopeStates.MAX_RETRACT),
    BETWEEN_MAX_RETRACT_AND_MAX_EXTENSION(DesiredTelescopeStates.DUMMY),
    MAX_EXTENSION(DesiredTelescopeStates.MAX_EXTENSION),
  }

  val telescopingTolerance = 1.inches

  val FORWARD_SOFT_LIMIT = 26.inches // old one was 24 inches
  val SLOW_TELESCOPING_THRESHOLD = 5.inches
  val REVERSE_SOFT_LIMIT = 0.inches

  const val TAB = "Telescoping Climber"
}
