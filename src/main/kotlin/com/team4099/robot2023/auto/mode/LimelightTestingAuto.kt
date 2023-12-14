package com.team4099.robot2023.auto.mode

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
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

class LimelightTestingAuto(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {

  init {
    addCommands(
      ResetPoseCommand(
        drivetrain, Pose2d(startingPosX.get(), startingPosY.get(), startingPosTheta.get())
      ),
      ParallelCommandGroup(
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                Translation2d(1.9.meters, 3.31.meters).translation2d,
                null,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(2.9.meters, 4.8.meters).translation2d,
                null,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(
                  FieldConstants.StagingLocations.translations[3]!!.x-intakeOffset.get(),
                  FieldConstants.StagingLocations.translations[3]!!.y-intakeOffset.get()
                )
                  .translation2d,
                0.0.degrees.inRotation2ds,
                0.0.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.groundIntakeConeCommand())
      ),
      ParallelCommandGroup(
        WaitCommand(1.0).andThen(superstructure.requestIdleCommand()),
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                Translation2d(
                  FieldConstants.StagingLocations.translations[3]!!.x- intakeOffset.get(),
                  FieldConstants.StagingLocations.translations[3]!!.y- intakeOffset.get()
                )
                  .translation2d,
                0.0.degrees.inRotation2ds,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(3.139.meters, 4.8.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(
                  endingPosX.get(),
                  FieldConstants.Grids.nodeFirstY +
                    FieldConstants.Grids.nodeSeparationY * 7
                )
                  .translation2d,
                null,
                180.0.degrees.inRotation2ds
              )
            )
          },
          keepTrapping = true
        )
      ),
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HIGH
      ),
      superstructure.score(),
      WaitCommand(0.5),
      ParallelCommandGroup(
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                Translation2d(1.9.meters, 3.31.meters).translation2d,
                null,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(2.9.meters, 4.8.meters).translation2d,
                null,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(
                  FieldConstants.StagingLocations.translations[2]!!.x-intakeOffset.get(),
                  FieldConstants.StagingLocations.translations[2]!!.y-intakeOffset.get()
                )
                  .translation2d,
                0.0.degrees.inRotation2ds,
                0.0.degrees.inRotation2ds
              ),
            )
          }
        ),
      WaitCommand(1.5).andThen(superstructure.groundIntakeConeCommand())
    ),
      ParallelCommandGroup(
        WaitCommand(1.0).andThen(superstructure.requestIdleCommand()),
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                Translation2d(
                  FieldConstants.StagingLocations.translations[2]!!.x- intakeOffset.get(),
                  FieldConstants.StagingLocations.translations[2]!!.y- intakeOffset.get()
                )
                  .translation2d,
                0.0.degrees.inRotation2ds,
                0.0.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(3.139.meters, 4.8.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              Waypoint(
                Translation2d(
                  endingPosX.get(),
                  FieldConstants.Grids.nodeFirstY +
                    FieldConstants.Grids.nodeSeparationY * 7
                )
                  .translation2d,
                null,
                180.0.degrees.inRotation2ds
              )
            )
          },
          keepTrapping = true
        )
      )
    )
  }

  companion object {
    val startingPosX =
      LoggedTunableValue(
        "Drivetrain/startingPosX1", 1.9.meters, Pair({ it.inMeters }, { it.meters })
      )
    val startingPosY =
      LoggedTunableValue(
        "Drivetrain/startingPosY1", 3.31.meters, Pair({ it.inMeters }, { it.meters })
      )
    val startingPosTheta =
      LoggedTunableValue(
        "Drivetrain/startingPosTheta1", 0.0.degrees, Pair({ it.inDegrees }, { it.degrees })
      )

    val endingPosX =
      LoggedTunableValue(
        "Drivetrain/endingPosX1", 1.9.meters, Pair({ it.inMeters }, { it.meters })
      )
    val endingPosY =
      LoggedTunableValue(
        "Drivetrain/endingPosY1", 3.31.meters, Pair({ it.inMeters }, { it.meters })
      )
    val endingPosTheta =
      LoggedTunableValue(
        "Drivetrain/endingPosTheta1", 180.0.degrees, Pair({ it.inDegrees }, { it.degrees })
      )
    val intakeOffset=
      LoggedTunableValue(
        "Drivetrain/intakeOffset", 0.0.meters, Pair({ it.inMeters }, { it.meters })
      )
  }
}
