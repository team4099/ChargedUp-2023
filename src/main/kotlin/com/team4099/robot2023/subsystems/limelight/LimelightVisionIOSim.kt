package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.toPose2d
import com.team4099.robot2023.util.toPose3d
import edu.wpi.first.networktables.NetworkTableInstance
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import kotlin.math.atan2

object LimelightVisionIOSim : LimelightVisionIO {

  val blueMidConeNodePoses = mutableListOf<Pose3d>()
  val blueHighConeNodePoses = mutableListOf<Pose3d>()

  val redMidConeNodePoses: MutableList<Pose3d>
  val redHighConeNodePoses: MutableList<Pose3d>
  init {
    for (nodeIndex in listOf(0, 2, 3, 5, 6, 8)) {
      blueMidConeNodePoses.add(
        Pose3d(
          FieldConstants.Grids.midX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * nodeIndex,
          VisionConstants.Limelight.MID_TAPE_HEIGHT,
          Rotation3d()
        )
      )
      blueHighConeNodePoses.add(
        Pose3d(
          FieldConstants.Grids.highX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * nodeIndex,
          VisionConstants.Limelight.HIGH_TAPE_HEIGHT,
          Rotation3d()
        )
      )
    }

    redMidConeNodePoses =
      blueMidConeNodePoses.map { AllianceFlipUtil.forceApply(it) }.toMutableList()
    redHighConeNodePoses =
      blueHighConeNodePoses.map { AllianceFlipUtil.forceApply(it) }.toMutableList()
  }

  private val simPose =
    NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/AdvantageKit/RealOutputs/${VisionConstants.POSE_TOPIC_NAME}")
      .getEntry(doubleArrayOf(0.0, 0.0, 0.0))

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    val cameraPose =
      simPose.get().toPose2d().toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM)
    Logger.getInstance().recordOutput("LimelightVision/cameraPose", cameraPose.pose3d)

    val visibleNodes = mutableListOf<Pose3d>()
    val readings = mutableListOf<LimelightReading>()
    for (
      node in
      blueMidConeNodePoses + blueHighConeNodePoses + redMidConeNodePoses + redHighConeNodePoses
    ) {
      val relativePose = node.relativeTo(cameraPose)

      // pitch (ty)
      val pitch =
        atan2(relativePose.z.absoluteValue.inMeters, relativePose.x.absoluteValue.inMeters)
          .radians // opposite over adjacent

      // yaw (tx)
      val yaw =
        atan2(relativePose.y.absoluteValue.inMeters, relativePose.x.absoluteValue.inMeters)
          .radians // opposite over adjacent

      if (pitch.absoluteValue <= VisionConstants.Limelight.VERITCAL_FOV / 2 &&
        yaw.absoluteValue <= VisionConstants.Limelight.HORIZONTAL_FOV / 2
      ) {
        readings.add(
          LimelightReading(
            -yaw, -pitch, 0.0, 0.0, 0.0.degrees
          )
        ) // TODO add view port stuff for finding pixel
        visibleNodes.add(node)
      }
    }

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/simVisibleNodes", *visibleNodes.map { it.pose3d }.toTypedArray()
      )

    inputs.timestamp = Clock.realTimestamp
    inputs.angle = 0.0.radians
    inputs.fps = 90.0
    inputs.validReading = true
    inputs.retroTargets = readings.toList()

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/blueMidNodes", *blueMidConeNodePoses.map { it.pose3d }.toTypedArray()
      )
    Logger.getInstance()
      .recordOutput(
        "LimelightVision/blueHighNodes",
        *blueHighConeNodePoses.map { it.pose3d }.toTypedArray()
      )

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/redMidNodes", *redMidConeNodePoses.map { it.pose3d }.toTypedArray()
      )
    Logger.getInstance()
      .recordOutput(
        "LimelightVision/redHighNodes", *redHighConeNodePoses.map { it.pose3d }.toTypedArray()
      )
  }
}
