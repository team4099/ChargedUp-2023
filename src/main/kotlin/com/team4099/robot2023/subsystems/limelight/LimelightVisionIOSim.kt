package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.toPose3d
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import java.lang.Math.atan

object LimelightVisionIOSim : LimelightVisionIO {

  var poseSupplier: () -> Pose2d = { Pose2d() }

  val poseToGamePiece: Pose3d
    get() {
      return Pose3d(
        FieldConstants.StagingLocations.positionX,
        FieldConstants.StagingLocations.translations[0]?.y ?: 0.meters,
        VisionConstants.Limelight.CONE_HEIGHT / 2,
        Rotation3d(0.0.degrees, 0.0.degrees, 0.0.degrees)
      )
        .relativeTo(
          poseSupplier.invoke().toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM)
        )
    }

  private val tunableTx: Angle
    get() {
      return atan(poseToGamePiece.y.inMeters / poseToGamePiece.x.inMeters).radians
    }

  private val tunableTy: Angle
    get() {
      return atan(poseToGamePiece.x.inMeters / poseToGamePiece.z.inMeters).radians
    }

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    inputs.timestamp = Clock.realTimestamp
    inputs.xAngle = 0.0.radians
    inputs.yAngle = 0.0.radians
    inputs.targetSize = 0.0.percent
    inputs.fps = 90.0
    inputs.validReading = true
    inputs.gamePieceTargets =
      listOf(LimelightReading("cone", 0.0.percent, tunableTx, tunableTy, 0.0, 0.0, 0.0.percent))
  }
}
