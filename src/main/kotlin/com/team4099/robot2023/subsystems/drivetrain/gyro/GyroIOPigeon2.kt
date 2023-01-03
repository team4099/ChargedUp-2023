package com.team4099.robot2023.subsystems.drivetrain.gyro

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.sensors.Pigeon2
import com.ctre.phoenix.sensors.Pigeon2Configuration
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GyroConstants

object GyroIOPigeon2 : GyroIO {
  private var pigeon2 = Pigeon2(Constants.Gyro.PIGEON_2_ID, Constants.Universal.CANIVORE_NAME)
  private val xyzDps = DoubleArray(3)

  init {
    val pigeon2Configuration = Pigeon2Configuration()
    pigeon2Configuration.MountPosePitch = GyroConstants.mountPitch.inRadians
    pigeon2Configuration.MountPoseYaw = GyroConstants.mountYaw.inRadians
    pigeon2Configuration.MountPoseRoll = GyroConstants.mountRoll.inRadians
    // TODO look into more pigeon configuration stuff
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    pigeon2.getRawGyro(xyzDps)

    inputs.gyroConnected = pigeon2.lastError.equals(ErrorCode.OK)

    inputs.gyroYaw = pigeon2.yaw.degrees
    inputs.gyroPitch = pigeon2.pitch.degrees
    inputs.gyroRoll = pigeon2.roll.degrees

    inputs.gyroVelocity = xyzDps[2].degrees.perSecond
  }
}
