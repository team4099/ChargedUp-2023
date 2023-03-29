package com.team4099.robot2023.util

import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees

data class LimelightReading(
  val tx: Angle,
  val ty: Angle,
  val txPixels: Double,
  val tyPixel: Double,
  val ts: Angle
) {

  constructor(
    limelightReading: LimelightHelpers.LimelightTarget_Retro
  ) : this(
    limelightReading.tx.degrees,
    limelightReading.ty.degrees,
    limelightReading.tx_pixels,
    limelightReading.ty_pixels,
    limelightReading.ts.degrees
  ) {}
}
