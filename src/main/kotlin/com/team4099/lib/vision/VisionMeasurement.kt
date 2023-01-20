package com.team4099.lib.vision

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time

data class VisionMeasurement(
  val timestamp: Time,
  val visionPose: Pose2d,
  val stdev: Triple<Double, Double, Double>
)
