package com.team4099.robot2023.subsystems.vision

import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.lib.vision.VisionMeasurement
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3dWPILIB

class Vision(cameras: VisionIO) : SubsystemBase() {
  val cameras = cameras.visionCameras

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  var visionMeasurements = mutableListOf<VisionMeasurement>()

  override fun periodic() {
    cameras.forEach { it.periodic() }

    val visibleTags: MutableList<Pose3dWPILIB> = mutableListOf()
    for (camera in cameras) {
      visibleTags += camera.detectedAprilTagIds.map { layout.getTagPose(it).pose3d }
    }

    val cameraVisionMeasurements = mutableListOf<VisionMeasurement>()
    for (camera in cameras) {
      for (detectedTag in 0 until camera.detectedAprilTagIds.size) {
        val tagPose = layout.getTagPose(camera.detectedAprilTagIds[detectedTag])
        cameraVisionMeasurements.add(
          VisionMeasurement(
            timestamp = camera.timestamp,
            visionPose =
            tagPose
              .transformBy(camera.bestTransforms[detectedTag].inverse())
              .transformBy(camera.io.transformToRobot.inverse())
              .toPose2d(), // length of known apriltags will always be equal to
            // bestTransforms
            stdev = camera.stdevs[detectedTag]
          )
        )
      }
    }

    visionMeasurements = cameraVisionMeasurements

    Logger.getInstance().recordOutput("Vision/VisibleTags", *(visibleTags.toTypedArray()))
    Logger.getInstance()
      .recordOutput(
        "Vision/bestPoses", *(visionMeasurements.map { it.visionPose.pose2d }.toTypedArray())
      )
  }
}
