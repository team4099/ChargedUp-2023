package com.team4099.lib.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d

object LogUtil {
  fun toPoseArray2d(pose: Pose2d): DoubleArray {
    return doubleArrayOf(
      pose.x,
      pose.y,
      pose.rotation.radians
    )
  }

  fun toPoseArray2d(poses: List<Pose2d>): DoubleArray {
    val result = DoubleArray(3 * poses.size)
    for (i in poses.indices) {
      val vals = toPoseArray2d(poses[i])
      for (j in vals.indices) {
        result[3 * i + j] = vals[j]
      }
    }
    return result
  }

  fun toPoseArray3d(pose: Pose3d): DoubleArray {
    val rot = pose.rotation
    return doubleArrayOf(
      pose.x,
      pose.y,
      pose.z,
      rot.quaternion.w,
      rot.quaternion.x,
      rot.quaternion.y,
      rot.quaternion.z
    )
  }

  @JvmStatic
  fun toPoseArray3d(poses: List<Pose3d>): DoubleArray {
    val result = DoubleArray(7 * poses.size)
    for (i in poses.indices) {
      val vals = toPoseArray3d(poses[i])
      for (j in vals.indices) {
        result[7 * i + j] = vals[j]
      }
    }
    return result
  }
}

