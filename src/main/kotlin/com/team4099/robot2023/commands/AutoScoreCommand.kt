package com.team4099.robot2023.commands

import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.config.constants.WaypointConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.gameboy.objective.isConeNode
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.perSecond

class AutoScoreCommand(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  lateinit var finalDrivePose: Pose2d
  lateinit var finalPose: Pose2d
  lateinit var postAlignPose: Pose2d
  lateinit var intermediaryWaypoints: List<Waypoint>
  var currentZone: Zone2d? = null
  var heading: Angle = 0.0.degrees
  lateinit var gamePiece: GamePiece
  lateinit var nodeTier: NodeTier

  init {

    val setupCommand =
      runOnce({
        Logger.getInstance().recordOutput("Auto/isAutoDriving", true)
        drivePose = drivetrain.odometryPose
        postAlignPose =
          AllianceFlipUtil.apply(
            Pose2d(
              1.8.meters, // slightly offset in the x
              FieldConstants.Grids.nodeFirstY +
                FieldConstants.Grids.nodeSeparationY *
                if (FMSData.isBlue) superstructure.objective.nodeColumn
                else 8 - superstructure.objective.nodeColumn,
              180.degrees
            )
          )

        finalPose =
          AllianceFlipUtil.apply(
            Pose2d(
              2.00.meters, // slightly offset in the x
              FieldConstants.Grids.nodeFirstY +
                FieldConstants.Grids.nodeSeparationY *
                (
                  if (FMSData.isBlue) superstructure.objective.nodeColumn
                  else 8 - superstructure.objective.nodeColumn
                  ),
              180.degrees
            )
          )

        heading = drivetrain.fieldVelocity.heading

        // Determine the current zone and calculate the trajectory
        currentZone = FieldConstants.determineZone(FieldConstants.Zones.allZones, drivePose)
        intermediaryWaypoints =
          WaypointConstants.CommunityPaths.getPath(currentZone).map {
            AllianceFlipUtil.apply(it)
          }

        println(currentZone == FieldConstants.Zones.closeCenterRightLane)

        gamePiece =
          if (superstructure.objective.isConeNode()) Constants.Universal.GamePiece.CONE
          else GamePiece.CUBE
        nodeTier = superstructure.objective.nodeTier
        Logger.getInstance().recordOutput("AutoScore/selectedGamePiece", gamePiece.name)
        Logger.getInstance().recordOutput("AutoScore/selectedNodeTier", nodeTier.name)
      })

    val breakCommand =
      runOnce({ drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(true) } })

    val coastCommand =
      runOnce({ drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(false) } })

    val updatedDrivePose = runOnce({ finalDrivePose = drivetrain.odometryPose })

    addCommands(
      setupCommand,
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              drivetrain.odometryPose.pose2d.translation,
              if (drivetrain.fieldVelocity.magnitude.absoluteValue < 0.25.meters.perSecond)
                null
              else heading.inRotation2ds,
              drivetrain.odometryPose.rotation.inRotation2ds
            )
          ) +
            // intermediaryWaypoints +
            listOf(
              Waypoint(
                finalPose.translation.translation2d,
                null,
                finalPose.rotation.inRotation2ds
              )
            )
        },
        keepTrapping = false,
        flipForAlliances = false,
        endVelocity = Velocity2d(0.meters.perSecond, 0.0.meters.perSecond),
        tolerance = Pose2d(5.inches, 3.inches, 3.degrees),
        forceRobotVelocityCheck = true
      ),
      updatedDrivePose,
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              finalDrivePose.pose2d.translation,
              null,
              finalDrivePose.rotation.inRotation2ds
            )
          ) +
            // intermediaryWaypoints +
            listOf(
              Waypoint(
                postAlignPose.translation.translation2d,
                null,
                postAlignPose.rotation.inRotation2ds
              )
            )
        },
        keepTrapping = false,
        flipForAlliances = false
      ),
      superstructure.prepScoreCommand({ gamePiece }, { nodeTier })
      //      ParallelCommandGroup(
      //        superstructure.prepScoreCommand({ gamePiece }, { nodeTier }),
      //        DrivePathCommand(
      //          drivetrain,
      //          {
      //            listOf(
      //              Waypoint(
      //                finalPose.pose2d.translation, null, finalPose.rotation.inRotation2ds
      //              ),
      //              Waypoint(
      //                postAlignPose.translation.translation2d,
      //                null,
      //                postAlignPose.rotation.inRotation2ds
      //              )
      //            )
      //          },
      //          keepTrapping = true,
      //          leaveOutYAdjustment = true,
      //          flipForAlliances = false
      //        )
      //          .withTimeout(0.5)
      //      )
    )
  }
}
