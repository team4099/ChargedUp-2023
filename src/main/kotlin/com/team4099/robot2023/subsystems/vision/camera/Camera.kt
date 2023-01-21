package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.vision.TargetCorner
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue

class Camera(val io: CameraIO) : SubsystemBase() {
  val inputs = CameraIO.CameraInputs()

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.apriltagsWpilib, Units.inchesToMeters(651.25), Units.inchesToMeters(315.5)
    )

  var detectedAprilTagIds = mutableListOf<Int>()

  var bestPoses = mutableListOf<Pose2d>()
  var altPoses = mutableListOf<Pose2d>()

  var timestamp = 0.0

  var stdevs = mutableListOf<Triple<Double, Double, Double>>()

  override fun periodic() {
    io.updateInputs(inputs)

    val corners = mutableListOf<TargetCorner>()
    val knownTags = mutableListOf<Int>()
    val bestPoseResult = mutableListOf<Pose2d>()
    val altPoseResult = mutableListOf<Pose2d>()
    val resultTimeStamp = inputs.visionResult.timestampSeconds

    for (target in inputs.visionResult.targets) {
      stdevs.add(
        Triple(
          Units.radiansToDegrees(target.bestTransform.rotation.x) / 3 +
            Units.radiansToDegrees(target.bestTransform.rotation.z).absoluteValue * 5,
          Units.radiansToDegrees(target.bestTransform.rotation.y) / 3 +
            Units.radiansToDegrees(target.bestTransform.rotation.z).absoluteValue * 5,
          1000.0
          // target.bestTransform.rotation.z.inDegrees.absoluteValue
        )
      )

      if (target.fiducialID in FieldConstants.apriltagsWpilib.map { it.ID }){
        knownTags.add(target.fiducialID)

        corners.addAll(target.targetCorners)

        val tagPose = layout.getTagPose(target.fiducialID)

        // getting transforms
        val camToBest = target.bestTransform
        val camToAlt = target.altTransform


        bestPoseResult.add(
          tagPose
            .get()
            .transformBy(camToBest.inverse())
            .transformBy(io.transformToRobot.inverse())
            .toPose2d()
        )
        altPoseResult.add(
          tagPose
            .get()
            .transformBy(camToAlt.inverse())
            .transformBy(io.transformToRobot.inverse())
            .toPose2d()
        )
      }
    }

    detectedAprilTagIds = knownTags
    bestPoses = bestPoseResult
    altPoses = altPoseResult
    timestamp = resultTimeStamp

    Logger.getInstance().recordOutput("/Vision/bestPoses", *(bestPoses.toTypedArray()))
    Logger.getInstance().recordOutput("/Vision/altPoses", *(altPoses.toTypedArray()))
  }
}
