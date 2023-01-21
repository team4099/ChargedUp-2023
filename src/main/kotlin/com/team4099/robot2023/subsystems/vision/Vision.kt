package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.vision.VisionMeasurement
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Vision(cameras: VisionIO) : SubsystemBase() {
  val cameras = cameras.visionCameras

  val layout: edu.wpi.first.apriltag.AprilTagFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      FieldConstants.apriltagsWpilib, Units.inchesToMeters(651.25), Units.inchesToMeters(315.5)
    )

  var visionMeasurements = mutableListOf<VisionMeasurement>()

  override fun periodic() {
    cameras.forEach { it.periodic() }

    val visibleTags: MutableList<Pose3d> = mutableListOf()
    val visibleTagIds: MutableList<Double> = mutableListOf()
    for (camera in cameras) {
      visibleTags += camera.detectedAprilTagIds.map { layout.getTagPose(it).get() }
      visibleTagIds += camera.detectedAprilTagIds.map { it.toDouble() }
    }

    val cameraVisionMeasurements = mutableListOf<VisionMeasurement>()
    for (camera in cameras) {
      for (detectedTag in 0 until camera.detectedAprilTagIds.size) {
        cameraVisionMeasurements.add(
          VisionMeasurement(
            timestamp = camera.timestamp,
            visionPose = camera.bestPoses[detectedTag],
            stdev = camera.stdevs[detectedTag]
          )
        )
      }
    }

    visionMeasurements = cameraVisionMeasurements

    Logger.getInstance().recordOutput("Vision/VisibleTags", *(visibleTags.toTypedArray()))
    Logger.getInstance()
      .recordOutput(
        "Vision/bestPoses", *(visionMeasurements.map { it.visionPose }.toTypedArray())
      )

    Logger.getInstance().recordOutput("Vision/visibleTagIDs", visibleTagIds.toDoubleArray())
  }
}
