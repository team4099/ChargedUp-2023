package com.team4099.robot2023.auto

import com.pathplanner.lib.PathPlanner
import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond

object PathStore {

  val testAutoPath =
    PathPlanner.loadPath(
      "strafeRight",
      DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond,
      DrivetrainConstants.MAX_AUTO_ACCEL.inMetersPerSecondPerSecond
    )
}
