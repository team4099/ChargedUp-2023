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

  //  val CAMERA_TRANSFORMS =
  //    listOf(
  //      Transform3d(
  //        Translation3d(12.89.inches, 7.154.inches, 29.33.inches),
  //        Rotation3d(180.degrees, -5.degrees, 0.degrees)
  //      ),
  //      Transform3d(
  //        Translation3d(-9.98.inches, -6.64.inches, 16.508.inches),
  //        Rotation3d(0.0.degrees, 0.0.degrees, 180.degrees)
  //      ),
  //      Transform3d(
  //        Translation3d(-10.036.inches, -12.793.inches, 16.438.inches),
  //        Rotation3d(0.0.degrees, 0.0.degrees, -40.degrees)
  //      ), // camera facing rightward
  //      Transform3d(
  //        Translation3d(-10.560.inches, 12.793.inches, 16.437.inches),
  //        Rotation3d(180.0.degrees, 0.0.degrees, 40.degrees)
  //      ) // camera facing leftward
  //    )

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(12.75.inches, 7.3125.inches, 28.75.inches), //18.69
        Rotation3d(180.degrees, 0.degrees, 0.degrees)
      ),
      //        Transform3d(
      //          Translation3d(-10.965.inches, -11.85.inches, 16.437.inches),
      //          Rotation3d(0.0.degrees, 0.0.degrees, 180.degrees)
      //        ),
      Transform3d(
        Translation3d(-6.560.inches, -13.575.inches, 16.25.inches),
        Rotation3d(0.0.degrees, 0.0.degrees, -40.degrees)
      ), // camera facing rightward
      Transform3d(
        Translation3d(-6.560.inches, 13.575.inches, 16.25.inches),
        Rotation3d(180.0.degrees, 0.0.degrees, 40.degrees)
      ) // camera facing leftward
    )

  val CAMERA_NAMES = listOf("northstar_1", "northstar_2", "northstar_3")

  object Limelight {
    val LIMELIGHT_NAME = "limelight-zapdos"
    val HORIZONTAL_FOV = 59.6.degrees
    val VERITCAL_FOV = 45.7.degrees
    val HIGH_TAPE_HEIGHT = 43.875.inches + 1.inches
    val MID_TAPE_HEIGHT = 23.905.inches + 1.inches
    val LL_TRANSFORM =
      Transform3d(
        Translation3d(1.1438.inches, 10.3966.inches, 12.9284.inches),
        Rotation3d(8.159.degrees, -90.degrees + 61.610.degrees, -14.1254.degrees)
      )
    const val RES_WIDTH = 320
    const val RES_HEIGHT = 240 // no clue what these numbers should be but usnig these for now
  }
}
