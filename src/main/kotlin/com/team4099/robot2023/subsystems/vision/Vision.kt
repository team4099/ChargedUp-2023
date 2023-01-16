package com.team4099.robot2023.subsystems.vision

import com.team4099.apriltag.AprilTag
import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import kotlin.math.absoluteValue

class Vision(val io: VisionIO) : SubsystemBase() {
  val inputs = VisionIO.VisionInputs()

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  var apriltagCorners = mutableListOf<TargetCorner>()
  var knownAprilTags = mutableListOf<AprilTag>()

  var bestPoses = mutableListOf<Pose2d>()
  var altPoses = mutableListOf<Pose2d>()

  var timestamps = mutableListOf<Time>()

  var stdevs = mutableListOf<Triple<Double, Double, Double>>()

  var previousResults = MutableList(VisionConstants.NUM_OF_CAMERAS) { PhotonPipelineResult() }

  override fun periodic() {
    io.updateInputs(inputs)

    val corners = mutableListOf<TargetCorner>()
    val knownTags = mutableListOf<AprilTag>()
    val bestCurPoses = mutableListOf<Pose2d>()
    val altCurPoses = mutableListOf<Pose2d>()
    var resultTimeStamps = mutableListOf<Time>()

    // each result corresponds to a camera
    for (resultIndex in 0 until inputs.photonResults.size) {
      if (inputs.photonResults[resultIndex].timestampSeconds.seconds ==
        previousResults[resultIndex].timestampSeconds.seconds
      ) {
        continue
      } else {
        previousResults[resultIndex] = inputs.photonResults[resultIndex]
      }

      for (target in inputs.photonResults[resultIndex].targets) {
        stdevs.add(
          Triple(
            1 / (0.01 * target.area) + (target.yaw - 90).absoluteValue / 90,
            1 / (0.01 * target.area) + (target.yaw - 90).absoluteValue / 90,
            (target.yaw - 90).absoluteValue / 100
          )
        )

        resultTimeStamps.add(inputs.photonResults[resultIndex].timestampSeconds.seconds)

        corners.addAll(target.corners)

        val tagPose = layout.getTagPose(target.fiducialId)
        knownTags.add(AprilTag(target.fiducialId, tagPose))

        // getting transforms
        val camToBest = Transform3d(target.bestCameraToTarget)
        val camToAlt = Transform3d(target.alternateCameraToTarget)

        // getting robot to camera
        val robotToCam: Transform3d = VisionConstants.CAMERA_TRANSFORMS[resultIndex]

        // getting poses
        val bestPose =
          tagPose.transformBy(camToBest.inverse()).transformBy(robotToCam.inverse()).toPose2d()

        val altPose =
          tagPose.transformBy(camToAlt.inverse()).transformBy(robotToCam.inverse()).toPose2d()

        // adding them back to the list
        bestCurPoses.add(bestPose)
        altCurPoses.add(altPose)
      }
    }

    previousResults = inputs.photonResults.map { it }.toMutableList() // copying all the elements

    apriltagCorners = corners
    knownAprilTags = knownTags
    bestPoses = bestCurPoses
    altPoses = altCurPoses
    timestamps = resultTimeStamps

    Logger.getInstance()
      .recordOutput("Vision/VisibleTags", *knownTags.map { it.pose.pose3d }.toTypedArray())

    Logger.getInstance()
      .recordOutput("Vision/bestPoses", *bestPoses.map { it.pose2d }.toTypedArray())
  }
}
