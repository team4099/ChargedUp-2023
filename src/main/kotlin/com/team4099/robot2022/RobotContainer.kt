package com.team4099.robot2022

import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.subsystems.vision.Vision
import com.team4099.robot2022.subsystems.vision.VisionIOHawkeye

object RobotContainer {
  private val vision: Vision

  init {
    if (Constants.Universal.ROBOT_MODE == Constants.Tuning.RobotType.REAL) {
      vision = Vision(VisionIOHawkeye)
    } else {
      vision = Vision(VisionIOHawkeye)
    }
  }
}
