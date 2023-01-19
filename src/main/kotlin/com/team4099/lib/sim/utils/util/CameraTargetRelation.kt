package com.team4099.lib.sim.utils.util

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d

/** Holds various helper geometries describing the relation between camera and target.  */
class CameraTargetRelation(val camPose: Pose3d, targetPose: Pose3d?) {
  @JvmField
  val camToTarg: Transform3d
  val camToTargDist: Double
  val camToTargDistXY: Double
  @JvmField
  val camToTargYaw: Rotation2d
  @JvmField
  val camToTargPitch: Rotation2d

  /** Angle from the camera's relative x-axis  */
  val camToTargAngle: Rotation2d
  val targToCam: Transform3d
  val targToCamYaw: Rotation2d
  val targToCamPitch: Rotation2d

  /** Angle from the target's relative x-axis  */
  @JvmField
  val targToCamAngle: Rotation2d

  init {
    camToTarg = Transform3d(camPose, targetPose)
    camToTargDist = camToTarg.translation.norm
    camToTargDistXY = Math.hypot(
      camToTarg.translation.x,
      camToTarg.translation.y
    )
    camToTargYaw = Rotation2d(camToTarg.x, camToTarg.y)
    camToTargPitch = Rotation2d(camToTargDistXY, -camToTarg.z)
    camToTargAngle = Rotation2d(
      Math.hypot(
        camToTargYaw.radians,
        camToTargPitch.radians
      )
    )
    targToCam = Transform3d(targetPose, camPose)
    targToCamYaw = Rotation2d(targToCam.x, targToCam.y)
    targToCamPitch = Rotation2d(camToTargDistXY, -targToCam.z)
    targToCamAngle = Rotation2d(
      Math.hypot(
        targToCamYaw.radians,
        targToCamPitch.radians
      )
    )
  }
}
