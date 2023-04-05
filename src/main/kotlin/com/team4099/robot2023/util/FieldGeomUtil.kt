package com.team4099.robot2023.util

import com.team4099.robot2023.config.constants.FieldConstants
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d

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
  if ((this.translation - pose1.translation).norm < (this.translation - pose2.translation).norm) {
    return pose1
  } else {
    return pose2
  }
}
