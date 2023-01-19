package com.team4099.lib.vision

import org.team4099.lib.units.base.Time

data class VisionResult(val latency: Time, val targets: List<VisionTarget>)
