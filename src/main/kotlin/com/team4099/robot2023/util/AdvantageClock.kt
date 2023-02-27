package com.team4099.robot2023.util

import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro

object AdvantageClock {
  val realTimestamp
    get() = Logger.getInstance().realTimestamp.micro.seconds
}
