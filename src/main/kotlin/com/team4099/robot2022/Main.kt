package com.team4099.robot2022

import edu.wpi.first.wpilibj.RobotBase
import kotlin.time.ExperimentalTime

class Main {
  companion object {
    @ExperimentalTime
    @JvmStatic
    fun main(args: Array<String>) {
      RobotBase.startRobot { Robot }
    }
  }
}
