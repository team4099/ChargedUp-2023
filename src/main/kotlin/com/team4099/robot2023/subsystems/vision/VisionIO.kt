package com.team4099.robot2023.subsystems.vision

import edu.wpi.first.apriltag.AprilTag
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose2d

interface VisionIO {
  class VisionInputs : LoggableInputs {
    var targetCorners = listOf<TargetCorner>()
    var knownVisTags = listOf<AprilTag>()
    var bestPoses = listOf<Pose2d>()
    var altPoses = listOf<Pose2d>()
    var photonResults = listOf<PhotonPipelineResult>()

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  fun updateInputs(inputs: VisionInputs) {}
}
