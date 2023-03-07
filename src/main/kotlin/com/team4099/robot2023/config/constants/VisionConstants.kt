package com.team4099.robot2023.config.constants

import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees

object VisionConstants {
  const val FRONT_CAMERA_NAME = "limelight"
  const val BACK_CAMERA_NAME = "thing2"
  const val SIDE_CAMERA_NAME = "thing3"

  const val SIM_POSE_TOPIC_NAME = "Odometry/groundTruthPose"
  const val POSE_TOPIC_NAME = "Odometry/pose"

  const val NUM_OF_CAMERAS = 1

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(4.756.inches, -7.4375.inches, 20.3768.inches),
        Rotation3d(180.degrees, 0.degrees, 0.degrees)
      )
    )

  val CAMERA_NAMES = listOf("Barry", "Wells", "Cisco", "Caitlin")
}
