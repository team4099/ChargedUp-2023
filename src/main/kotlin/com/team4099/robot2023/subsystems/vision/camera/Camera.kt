package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.lib.vision.TargetCorner
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import kotlin.math.absoluteValue

class Camera(val io: CameraIO) : SubsystemBase() {
  val inputs = CameraIO.CameraInputs()

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  var detectedAprilTagIds = mutableListOf<Int>()

  var bestPoses = mutableListOf<Pose2d>()
  var altPoses = mutableListOf<Pose2d>()

  var timestamp: Time = 0.0.seconds

  var stdevs = mutableListOf<Triple<Double, Double, Double>>()

  override fun periodic() {
    io.updateInputs(inputs)

    val corners = mutableListOf<TargetCorner>()
    val knownTags = mutableListOf<Int>()
    val bestPoseResult = mutableListOf<Pose2d>()
    val altPoseResult = mutableListOf<Pose2d>()
    val resultTimeStamp: Time = inputs.visionResult.timestamp

    for (target in inputs.visionResult.targets) {
      stdevs.add(
        Triple(
          1 / (0.01 * target.area) + (target.yaw - 90.degrees).inDegrees.absoluteValue / 90,
          1 / (0.01 * target.area) + (target.yaw - 90.degrees).inDegrees.absoluteValue / 90,
          (target.yaw - 90.degrees).inDegrees.absoluteValue / 100
        )
      )
      knownTags.add(target.fiducialID)

      corners.addAll(target.targetCorners)

      val tagPose = layout.getTagPose(target.fiducialID)

      // getting transforms
      val camToBest = target.bestTransform
      val camToAlt = target.altTransform

      bestPoseResult.add(
        tagPose
          .transformBy(camToBest.inverse())
          .transformBy(io.transformToRobot.inverse())
          .toPose2d()
      )
      altPoseResult.add(
        tagPose
          .transformBy(camToAlt.inverse())
          .transformBy(io.transformToRobot.inverse())
          .toPose2d()
      )
    }

    detectedAprilTagIds = knownTags
    bestPoses = bestPoseResult
    altPoses = altPoses
    timestamp = resultTimeStamp
  }
}
