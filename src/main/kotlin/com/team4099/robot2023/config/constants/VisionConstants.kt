package com.team4099.robot2023.config.constants

import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.derived.degrees

object VisionConstants {
  const val CAMERA_NAME = "photoncamera"

  val CAMERA_HEIGHT = 29.413.inches // TODO: Update with correct value
  val UPPER_HUB_TARGET_HEIGHT = (8.feet + 8.inches) // TODO: Make sure this is correct
  val CAMERA_ANGLE = 60.degrees

  val TARGET_RANGE = 5.inches // TODO: Update with correct value
  val RANGE_THRESHOLD = 2.inches // TODO: Update with correct value
}
