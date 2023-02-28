package com.team4099.robot2023.auto

import com.team4099.lib.pathfollow.TrajectoryConfig
import com.team4099.robot2023.config.constants.DrivetrainConstants

object PathStore {
  val trajectoryConfig =
    TrajectoryConfig(
      DrivetrainConstants.MAX_AUTO_VEL,
      DrivetrainConstants.MAX_AUTO_ACCEL,
      DrivetrainConstants.MAX_AUTO_ANGULAR_VEL,
      DrivetrainConstants.MAX_AUTO_ANGULAR_ACCEL
    )
}
