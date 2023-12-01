package com.team4099.robot2023.util

import com.team4099.utils.LimelightHelpers.LimelightTarget_Detector
import org.team4099.lib.units.base.Decimal
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees

data class LimelightReading(
  val className: String,
  val confidence: Decimal,
  val tx: Angle,
  val ty: Angle,
  val txPixel: Double,
  val tyPixel: Double,
  val ta: Decimal
) {

  constructor(
    limelightReading: LimelightTarget_Detector
  ) : this(
    limelightReading.className,
    limelightReading.confidence.percent,
    limelightReading.tx.degrees,
    limelightReading.ty.degrees,
    limelightReading.tx_pixels,
    limelightReading.ty_pixels,
    limelightReading.ta.percent
  )
}
