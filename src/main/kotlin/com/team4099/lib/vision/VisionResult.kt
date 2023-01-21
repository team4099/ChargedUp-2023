package com.team4099.lib.vision

data class VisionResult(
  val latencyMS: Double = 0.0,
  val timestampSeconds: Double = 0.0,
  val targets: List<VisionTarget> = listOf()
)
