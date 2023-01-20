package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.vision.VisionResult
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.geometry.Transform3d

interface CameraIO {
  class CameraInputs : LoggableInputs {
    var visionResult = VisionResult()
    var hasTargets = false

    override fun toLog(table: LogTable?) {
      table?.put("hasTargets", hasTargets)
    }

    override fun fromLog(table: LogTable?) {
      table?.getBoolean("hasTargets", hasTargets)?.let { hasTargets = it }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}

  val cameraName: String

  val transformToRobot: Transform3d
}
