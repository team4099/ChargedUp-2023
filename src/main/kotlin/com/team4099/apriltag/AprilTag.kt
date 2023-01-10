package com.team4099.apriltag

import edu.wpi.first.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d

class AprilTag(val id: Int, val pose: Pose3d) {
  constructor(apriltagWpilib: AprilTag) : this(apriltagWpilib.ID, Pose3d(apriltagWpilib.pose)) {}

  val apriltagWpilib = AprilTag(id, pose.pose3d)

  override fun equals(obj: Any?): Boolean {
    if (obj is com.team4099.apriltag.AprilTag) {
      val other = obj
      return id == other.id && pose == other.pose
    }
    return false
  }
}
