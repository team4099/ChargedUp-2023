package com.team4099.robot2023.util

import edu.wpi.first.wpilibj.DriverStation

object FMSData {
  var allianceColor = DriverStation.Alliance.Invalid

  val isBlue: Boolean
    get() = DriverStation.getAlliance() == DriverStation.Alliance.Blue
}
