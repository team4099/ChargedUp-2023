package com.team4099.lib.vision

import org.photonvision.targeting.TargetCorner

data class TargetCorner(val x: Double, val y: Double) {
  constructor(
    photonTargetCorner: TargetCorner
  ) : this(photonTargetCorner.x, photonTargetCorner.y) {}
}
