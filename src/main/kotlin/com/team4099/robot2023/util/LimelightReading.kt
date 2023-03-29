package com.team4099.robot2023.util

import com.team4099.utils.LimelightHelpers.LimelightTarget_Retro
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees

data class LimelightReading(
  val tx: Angle,
  val ty: Angle,
  val txPixel: Double,
  val tyPixel: Double,
  val ts: Angle
) {

  constructor(
    limelightReading: LimelightTarget_Retro
  ) : this(
    limelightReading.tx.degrees,
    limelightReading.ty.degrees,
    limelightReading.tx_pixels,
    limelightReading.ty_pixels,
    limelightReading.ts.degrees
  ) {}
}
