package com.team4099.robot2022.config.constants

import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.perSecond

object ElevatorConstants {
  val MAX_HEIGHT_OF_TOP_OF_CARRIAGE = 78.0.inches
  val BOTTOM_HEIGHT_OF_TOP_OF_CARRIAGE = 68.0.inches
  val MIDDLE_HEIGHT_OF_TOP_OF_CARRIAGE = 73.0.inches

  val ELEVATOR_KS = 0.0.volts // TODO tune values
  val ELEVATOR_KV = 0.0.volts / 1.0.meters.perSecond // TODO tune values
  val ELEVATOR_KA = 0.0.volts / 1.0.meters.perSecond.perSecond // TODO tune values
  val ELEVATOR_KG = 0.0.volts // TODO tune values
}
