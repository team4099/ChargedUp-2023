package com.team4099.robot2023.auto.mode

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.PositionAutoLevel
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
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

class ConeCubeLowOverChargeStationAuto(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : SequentialCommandGroup() {

  init {
    addCommands(
      ResetPoseCommand(
        drivetrain, Pose2d(startingPosX.get(), startingPosY.get(), startingPosTheta.get())
      ),
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HIGH
      ),
      superstructure.score(),
      ParallelCommandGroup(
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                Translation2d(1.9.meters, 2.21.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(3.8.meters, 2.8.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(5.35.meters, 2.15.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(6.5.meters, 2.15.meters).translation2d,
                null,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(
                  FieldConstants.StagingLocations.translations[1]!!.x,
                  FieldConstants.StagingLocations.translations[1]!!.y
                )
                  .translation2d,
                null,
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
            Waypoint(
              Translation2d(
                FieldConstants.StagingLocations.translations[1]!!.x,
                FieldConstants.StagingLocations.translations[1]!!.y
              )
                .translation2d,
              180.degrees.inRotation2ds,
              -45.0.degrees.inRotation2ds
            ),
            Waypoint(
              Translation2d(5.4.meters, 2.0.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ),
            Waypoint(
              Translation2d(
                endingPosX.get(),
                FieldConstants.Grids.nodeFirstY +
                  FieldConstants.Grids.nodeSeparationY * 4
              )
                .translation2d,
              null,
              180.0.degrees.inRotation2ds
            )
          )
        },
        keepTrapping = true
      ),
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HYBRID
      ),
      superstructure.score(),
      PositionAutoLevel(drivetrain)
    )
  }

  companion object {
    val startingPosX =
      LoggedTunableValue(
        "Drivetrain/startingPosX", 1.9.meters, Pair({ it.inMeters }, { it.meters })
      )
    val startingPosY =
      LoggedTunableValue(
        "Drivetrain/startingPosY", 2.21.meters, Pair({ it.inMeters }, { it.meters })
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
        "Drivetrain/endingPosY", 4.97.meters, Pair({ it.inMeters }, { it.meters })
      )
    val endingPosTheta =
      LoggedTunableValue(
        "Drivetrain/endingPosTheta", 180.0.degrees, Pair({ it.inDegrees }, { it.degrees })
      )
  }
}
