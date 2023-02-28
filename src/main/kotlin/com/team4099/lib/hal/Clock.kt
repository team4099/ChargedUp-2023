package com.team4099.lib.hal

import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro

object Clock {
  val fpgaTime: Time
    get() = RobotController.getFPGATime().micro.seconds

  val realTimestamp
    get() = Logger.getInstance().realTimestamp.micro.seconds
}
