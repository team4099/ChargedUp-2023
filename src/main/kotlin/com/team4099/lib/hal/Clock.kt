package com.team4099.lib.hal

import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.micro
import edu.wpi.first.wpilibj.RobotController

object Clock {
  val fpgaTime: Time
    get() = RobotController.getFPGATime().micro.seconds
}
