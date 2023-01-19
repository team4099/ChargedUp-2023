package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.apriltag.AprilTag
import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import kotlin.math.absoluteValue

class Camera(val io: CameraIO): SubsystemBase() {
  val inputs = CameraIO.CameraInputs()

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  var knownAprilTags = mutableListOf<AprilTag>()

  var bestTransforms = mutableListOf<Transform3d>()
  var altTransforms = mutableListOf<Transform3d>()

  var timestamp: Time = 0.0.seconds

  var stdevs = mutableListOf<Triple<Double, Double, Double>>()

  override fun periodic() {
    io.updateInputs(inputs)

    val corners = mutableListOf<TargetCorner>()
    val knownTags = mutableListOf<AprilTag>()
    val bestTransformResult = mutableListOf<Transform3d>()
    val altTransformResult = mutableListOf<Transform3d>()
    val resultTimeStamp: Time = inputs.photonResult.timestampSeconds.seconds

    for (target in inputs.photonResult.targets) {
      stdevs.add(
        Triple(
          1 / (0.01 * target.area) + (target.yaw - 90).absoluteValue / 90,
          1 / (0.01 * target.area) + (target.yaw - 90).absoluteValue / 90,
          (target.yaw - 90).absoluteValue / 100
        )
      )

      corners.addAll(target.corners)

      val tagPose = layout.getTagPose(target.fiducialId)
      knownTags.add(AprilTag(target.fiducialId, tagPose))

      // getting transforms
      val camToBest = Transform3d(target.bestCameraToTarget)
      val camToAlt = Transform3d(target.alternateCameraToTarget)

      bestTransformResult.add(camToBest)
      altTransformResult.add(camToAlt)
    }

    knownAprilTags = knownTags
    bestTransforms = bestTransformResult
    altTransforms = altTransformResult
    timestamp = resultTimeStamp
  }
}
