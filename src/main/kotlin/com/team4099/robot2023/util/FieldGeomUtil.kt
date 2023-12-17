package com.team4099.robot2023.util

import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d

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
