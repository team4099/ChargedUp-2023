package com.team4099.robot2023.subsystems.drivetrain.gyro

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.sensors.Pigeon2
import com.ctre.phoenix.sensors.Pigeon2Configuration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.inDegreesPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.GyroConstants
import kotlin.math.IEEErem

object GyroIOPigeon2 : GyroIO {
  private var pigeon2 = Pigeon2(Constants.Gyro.PIGEON_2_ID, Constants.Universal.CANIVORE_NAME)
  private val xyzDps = DoubleArray(3)

  val isConnected = pigeon2.lastError.equals(ErrorCode.OK)

  var gyroYawOffset: Angle = 0.0.degrees
  var gyroPitchOffset: Angle = 0.0.degrees
  var gyroRollOffset: Angle = 0.0.degrees

  /** The current angle of the drivetrain. */
  val gyroYaw: Angle
    get() {
      if (isConnected) {
        var rawYaw = pigeon2.yaw + gyroYawOffset.inDegrees
        rawYaw += DrivetrainConstants.GYRO_RATE_COEFFICIENT * gyroYawRate.inDegreesPerSecond
        return rawYaw.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroPitch: Angle
    get() {
      if (isConnected) {
        val rawPitch = pigeon2.pitch + gyroPitchOffset.inDegrees
        return rawPitch.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroRoll: Angle
    get() {
      if (isConnected) {
        val rawRoll = pigeon2.roll + gyroRollOffset.inDegrees
        return rawRoll.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
      }
    }

  val gyroYawRate: AngularVelocity
    get() {
      if (isConnected) {
        return xyzDps[2].degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroPitchRate: AngularVelocity
    get() {
      if (isConnected) {
        return xyzDps[1].degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroRollRate: AngularVelocity
    get() {
      if (isConnected) {
        return xyzDps[0].degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  init {
    val pigeon2Configuration = Pigeon2Configuration()
    pigeon2Configuration.MountPosePitch = GyroConstants.mountPitch.inRadians
    pigeon2Configuration.MountPoseYaw = GyroConstants.mountYaw.inRadians
    pigeon2Configuration.MountPoseRoll = GyroConstants.mountRoll.inRadians

    // TODO look into more pigeon configuration stuff
    pigeon2.configAllSettings(pigeon2Configuration)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    pigeon2.getRawGyro(xyzDps) // calling this here so it updated xyzDps which is called upon later

    inputs.gyroConnected = isConnected

    inputs.gyroYaw = gyroYaw
    inputs.gyroPitch = gyroPitch
    inputs.gyroRoll = gyroRoll

    inputs.gyroYawRate = gyroYawRate
    inputs.gyroPitchRate = gyroPitchRate
    inputs.gyroRollRate = gyroRollRate
  }

  override fun zeroGyroYaw(toAngle: Angle) {
    gyroYawOffset = toAngle - gyroYaw
  }

  override fun zeroGyroPitch(toAngle: Angle) {
    gyroPitchOffset = toAngle - gyroPitch
  }

  override fun zeroGyroRoll(toAngle: Angle) {
    gyroRollOffset = toAngle - gyroRoll
  }
}
