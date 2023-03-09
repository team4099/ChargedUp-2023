package com.team4099.robot2023.auto.mode

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds

class ConeCubeBumpAuto(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {

  init {
    addCommands(
      ResetPoseCommand(
        drivetrain, Pose2d(startingPosX.get(), startingPosY.get(), startingPosTheta.get())
      ),
//      superstructure.prepScoreCommand(
//        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HIGH
//      ),
//      superstructure.score(),
      ParallelCommandGroup(
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              // initial waypoint
              Waypoint(
                AllianceFlipUtil.apply(
                  Translation2d(startingPosX.get(), startingPosY.get())
                )
                  .translation2d,
                null,
                180.0.degrees.inRotation2ds
              ),
              // middle of bump
              Waypoint(
                AllianceFlipUtil.apply(Translation2d(4.59.meters, 0.65.meters))
                  .translation2d,
                null,
                180.0.degrees.inRotation2ds
              ),
              // pick up cube
              Waypoint(
                AllianceFlipUtil.apply(
                  Translation2d(
                    FieldConstants.StagingLocations.translations[0]!!.x,
                    FieldConstants.StagingLocations.translations[0]!!.y
                  )
                )
                  .translation2d,
                0.0.degrees.inRotation2ds,
                0.0.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(2.0).andThen(superstructure.groundIntakeCubeCommand())
      ),
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            // initial @ cube
            Waypoint(
              AllianceFlipUtil.apply(
                Translation2d(
                  FieldConstants.StagingLocations.translations[0]!!.x,
                  FieldConstants.StagingLocations.translations[0]!!.y
                )
              )
                .translation2d,
              180.0.degrees.inRotation2ds,
              0.0.degrees.inRotation2ds
            ),
            // middle of bump
            Waypoint(
              AllianceFlipUtil.apply(Translation2d(5.26.meters, 0.65.meters)).translation2d,
              null,
              180.0.degrees.inRotation2ds
            ),
            // scoring cube
            Waypoint(
              AllianceFlipUtil.apply(Translation2d(endingPosX.get(), endingPosY.get()))
                .translation2d,
              null,
              180.0.degrees.inRotation2ds
            )
          )
        },
        keepTrapping = true
      ),
//      superstructure.prepScoreCommand(
//        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HIGH
//      ),
//      superstructure.score()
    )
  }

  companion object {
    val startingPosX =
      LoggedTunableValue(
        "Drivetrain/startingPosX", 1.9.meters, Pair({ it.inMeters }, { it.meters })
      )
    val startingPosY =
      LoggedTunableValue(
        "Drivetrain/startingPosY", 0.57.meters, Pair({ it.inMeters }, { it.meters })
      )
    val startingPosTheta =
      LoggedTunableValue(
        "Drivetrain/startingPosTheta", 180.0.degrees, Pair({ it.inDegrees }, { it.degrees })
      )

    val endingPosX =
      LoggedTunableValue(
        "Drivetrain/endingPosX", 1.9.meters, Pair({ it.inMeters }, { it.meters })
      )
    val endingPosY =
      LoggedTunableValue(
        "Drivetrain/endingPosY",
        FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY,
        Pair({ it.inMeters }, { it.meters })
      )
    val endingPosTheta =
      LoggedTunableValue(
        "Drivetrain/endingPosTheta", 180.0.degrees, Pair({ it.inDegrees }, { it.degrees })
      )
  }
}
