package com.team4099.robot2023.auto

import com.team4099.lib.pathfollow.Path
import com.team4099.lib.pathfollow.TrajectoryConfig
import com.team4099.lib.pathfollow.Velocity2d
import com.team4099.lib.pathfollow.trajectoryFromPath
import com.team4099.robot2023.auto.mode.CoCuAuto
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.FieldConstants
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond

object PathStore {
  val trajectoryConfig =
    TrajectoryConfig(
      DrivetrainConstants.MAX_AUTO_VEL,
      DrivetrainConstants.MAX_AUTO_ACCEL,
      DrivetrainConstants.MAX_AUTO_ANGULAR_VEL,
      DrivetrainConstants.MAX_AUTO_ANGULAR_ACCEL
    )

  val rightNodeToFirstStage = trajectoryFromPath(
    Path(
      Pose2d(CoCuAuto.startingPosX.get(), CoCuAuto.startingPosY.get(), CoCuAuto.startingPosTheta.get()),
      Pose2d(FieldConstants.StagingLocations.translations[3]!!.x, FieldConstants.StagingLocations.translations[3]!!.y, 0.0.degrees),
      Velocity2d(),
      Velocity2d(1.0.meters.perSecond, 0.0.meters.perSecond)
    ),
    trajectoryConfig
  )

  val firstStagetoRightNode = trajectoryFromPath(
    Path(
      Pose2d(FieldConstants.StagingLocations.translations[3]!!.x, FieldConstants.StagingLocations.translations[3]!!.y, 0.0.degrees),
      Pose2d(CoCuAuto.endingPosX.get(), CoCuAuto.endingPosY.get(), CoCuAuto.endingPosTheta.get()),
      Velocity2d(1.0.meters.perSecond, 0.0.meters.perSecond),
      Velocity2d()
    ),
    trajectoryConfig
  )
}
