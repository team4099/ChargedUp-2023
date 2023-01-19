package com.team4099.robot2023.subsystems.vision.camera

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.targeting.PhotonPipelineResult

interface CameraIO {
  class CameraInputs: LoggableInputs{
    val photonResult = PhotonPipelineResult()

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

  }

  fun updateInputs(inputs: CameraInputs) {}
}
