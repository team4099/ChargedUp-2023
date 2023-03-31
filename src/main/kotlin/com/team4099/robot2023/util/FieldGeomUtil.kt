package com.team4099.robot2023.util

import com.team4099.robot2023.config.constants.FieldConstants
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.inMeters
import kotlin.math.pow
import kotlin.math.sqrt

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

fun Pose3d.findClosestPose(vararg pose3d: Pose3d): Pose3d {
  var closestPose = pose3d[0]
  if (pose3d.size > 1) {
    for (pose in pose3d) {
      if (this.closerToInTranslation(pose, closestPose) == pose) {
        closestPose = pose
      }
    }
  }

  return closestPose
}

fun Transform3d.toPose3d(): Pose3d {
  return Pose3d(this.translation, this.rotation)
}

fun Pose3d.toTransform3d(): Transform3d {
  return Transform3d(this.translation, this.rotation)
}

fun Pose3d.closerToInTranslation(pose1: Pose3d, pose2: Pose3d): Pose3d {
  val distToPose1 =
    sqrt(
      (this.x - pose1.x).inMeters.pow(2) +
        (this.y - pose1.y).inMeters.pow(2) +
        (this.z - pose1.z).inMeters.pow(2)
    )
  val distToPose2 =
    sqrt(
      (this.x - pose2.x).inMeters.pow(2) +
        (this.y - pose2.y).inMeters.pow(2) +
        (this.z - pose2.z).inMeters.pow(2)
    )
  if (distToPose1 < distToPose2) {
    return pose1
  } else {
    return pose2
  }
}
