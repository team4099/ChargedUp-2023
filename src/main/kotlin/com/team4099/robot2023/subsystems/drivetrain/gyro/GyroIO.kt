package com.team4099.robot2023.subsystems.drivetrain.gyro

import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inDegreesPerSecond
import com.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var gyroYaw = 0.0.radians
    var gyroPitch = 0.0.radians
    var gyroRoll = 0.0.radians
    var gyroYawRate = 0.0.radians.perSecond
    var gyroPitchRate = 0.0.radians.perSecond
    var gyroRollRate = 0.0.radians.perSecond

    var gyroConnected = false

    override fun toLog(table: LogTable?) {
      table?.put("gyroYawAngleDegrees", gyroYaw.inDegrees)
      table?.put("gyroPitchAngleDegrees", gyroPitch.inDegrees)
      table?.put("gyroRollAngleDegrees", gyroRoll.inDegrees)
      table?.put("gyroYawRateDegreesPerSecond", gyroYawRate.inDegreesPerSecond)
      table?.put("gyroPitchRateDegreesPerSecond", gyroPitchRate.inDegreesPerSecond)
      table?.put("gyroRollRateDegreesPerSecond", gyroRollRate.inDegreesPerSecond)
      table?.put("gyroConnected", gyroConnected)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("gyroAngleDegrees", gyroYaw.inDegrees)?.let { gyroYaw = it.degrees }
      table?.getDouble("gyroPitchDegrees", gyroPitch.inDegrees)?.let { gyroPitch = it.degrees }
      table?.getDouble("gyroRollDegrees", gyroRoll.inDegrees)?.let { gyroRoll = it.degrees }
      table?.getDouble("gyroYawRateDegreesPerSecond", gyroYawRate.inDegreesPerSecond)?.let {
        gyroYawRate = it.degrees.perSecond
      }
      table?.getDouble("gyroPitchRateDegreesPerSecond", gyroPitchRate.inDegreesPerSecond)?.let {
        gyroPitchRate = it.degrees.perSecond
      }
      table?.getDouble("gyroRollRateDegreesPerSecond", gyroRollRate.inDegreesPerSecond)?.let {
        gyroRollRate = it.degrees.perSecond
      }
      table?.getBoolean("gyroConnected", gyroConnected)?.let { gyroConnected = it }
    }
  }
  fun updateInputs(inputs: GyroIOInputs) {}

  fun zeroGyroYaw(toAngle: Angle) {}

  fun zeroGyroPitch(toAngle: Angle) {}

  fun zeroGyroRoll(toAngle: Angle) {}
}
