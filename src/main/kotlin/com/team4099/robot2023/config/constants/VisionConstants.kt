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
        Translation3d((11.760.inches - 1.25.inches), 7.3125.inches, 29.33.inches),
        Rotation3d(180.degrees, 0.degrees, 0.degrees)
      )
    )

  val CAMERA_NAMES = listOf("northstar")

  object Limelight {
    val HORIZONTAL_FOV = 59.6.degrees
    val VERITCAL_FOV = 45.7.degrees
    val LL_TRANSFORM =
      Transform3d(
        Translation3d((11.760.inches - 1.25.inches), 7.3125.inches, 29.33.inches),
        Rotation3d(180.degrees, 0.degrees, 0.degrees)
      )
    val RES_WIDTH = 320
    val RES_HEIGHT = 240 // no clue what these numbers should be but usnig these for now
  }
}
