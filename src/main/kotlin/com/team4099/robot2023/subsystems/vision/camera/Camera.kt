package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.lib.vision.TargetCorner
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.seconds
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
          target.bestTransform.x.inMeters / 3 +
            target.bestTransform.rotation.z.inDegrees.absoluteValue * 5,
          target.bestTransform.y.inMeters / 3 +
            target.bestTransform.rotation.z.inDegrees.absoluteValue * 5,
          1000.0
          // target.bestTransform.rotation.z.inDegrees.absoluteValue
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
    altPoses = altPoseResult
    timestamp = resultTimeStamp

    Logger.getInstance()
      .recordOutput("/Vision/bestPoses", *(bestPoses.map { it.pose2d }.toTypedArray()))
    Logger.getInstance()
      .recordOutput("/Vision/altPoses", *(altPoses.map { it.pose2d }.toTypedArray()))
  }
}
