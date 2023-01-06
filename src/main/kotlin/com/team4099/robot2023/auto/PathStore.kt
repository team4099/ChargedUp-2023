package com.team4099.robot2023.auto

import com.pathplanner.lib.PathPlanner
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.robot2023.config.constants.DrivetrainConstants

object PathStore {

  val testAutoPath =
    PathPlanner.loadPath(
      "strafeRight",
      DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond,
      DrivetrainConstants.MAX_AUTO_ACCEL.inMetersPerSecondPerSecond
    )
}
