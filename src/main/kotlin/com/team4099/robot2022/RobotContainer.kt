package com.team4099.robot2022

import com.team4099.robot2022.config.constants.Constants

object RobotContainer {

  init {
    if (Constants.Universal.ROBOT_MODE == Constants.Tuning.RobotType.REAL) {
      // Real Hardware Implementations
    } else {
      // Simulation implementations
    }
  }

  fun mapDefaultCommands() {
    TODO()
  }

  fun mapTeleopControls() {
    TODO()
  }
}
