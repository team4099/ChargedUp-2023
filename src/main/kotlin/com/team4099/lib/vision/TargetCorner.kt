package com.team4099.lib.vision

import com.team4099.robot2023.subsystems.limelight.CoordinatePair
import org.photonvision.targeting.TargetCorner

data class TargetCorner(val x: Double, val y: Double) {
  constructor(
    photonTargetCorner: TargetCorner
  ) : this(photonTargetCorner.x, photonTargetCorner.y) {}

  fun toCoordinatePair(): CoordinatePair {
    return CoordinatePair(x, y)
  }
}
