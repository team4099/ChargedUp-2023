package com.team4099.robot2023.util

import com.team4099.robot2023.config.constants.FieldConstants
import org.team4099.lib.geometry.Pose2d

fun Pose2d.isOnInnerSideOfChargeStation(): Boolean {
  val normalizedPose = AllianceFlipUtil.apply(this)
  return normalizedPose.x < FieldConstants.Community.chargingStationInnerX
}

fun Pose2d.isAboveMiddleOfChargeStation(): Boolean {
  val normalizedPose = AllianceFlipUtil.apply(this)
  return normalizedPose.y >
    (
      (
        FieldConstants.Community.chargingStationLeftY +
          FieldConstants.Community.chargingStationRightY
        ) / 2
      )
}
