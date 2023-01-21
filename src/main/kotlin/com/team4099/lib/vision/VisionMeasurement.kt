package com.team4099.lib.vision

import edu.wpi.first.math.geometry.Pose2d

data class VisionMeasurement(
  val timestamp: Double,
  val visionPose: Pose2d,
  val stdev: Triple<Double, Double, Double>
)
