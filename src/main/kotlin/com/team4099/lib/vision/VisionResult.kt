package com.team4099.lib.vision

import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds

data class VisionResult(
  val latency: Time = 0.0.seconds,
  val timestamp: Time = 0.0.seconds,
  val targets: List<VisionTarget> = listOf()
)
