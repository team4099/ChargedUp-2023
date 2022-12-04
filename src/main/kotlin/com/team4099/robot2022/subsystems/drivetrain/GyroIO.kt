package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var gyroYaw = 0.0.radians
    var gyroPitch = 0.0.radians
    var gyroRoll = 0.0.radians
    var gyroVelocity = 0.0.radians.perSecond

    var gyroConnected = false
    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }
}
