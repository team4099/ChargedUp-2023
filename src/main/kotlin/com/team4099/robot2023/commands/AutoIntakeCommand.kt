package com.team4099.robot2023.commands

import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.config.constants.WaypointConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.perSecond

class AutoIntakeCommand(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  lateinit var intermediaryWaypoints: List<Waypoint>
  var currentZone: Zone2d? = null
  lateinit var finalPose: Pose2d
  lateinit var postAlignPose: Pose2d
  var heading: Angle = 0.0.degrees
  lateinit var gamePiece: GamePiece
  lateinit var nodeTier: NodeTier

  init {
    val setupCommand =
      runOnce({
        Logger.getInstance().recordOutput("Auto/isAutoDriving", true)
        drivePose = drivetrain.odometryPose
        heading = drivetrain.fieldVelocity.heading
        currentZone = FieldConstants.determineZone(FieldConstants.Zones.allZones, drivePose)
        println(currentZone)
        intermediaryWaypoints =
          WaypointConstants.SubstationPaths.getPath(currentZone).map {
            AllianceFlipUtil.apply(it)
          }
        if (intermediaryWaypoints.isEmpty()) {
          intermediaryWaypoints =
            listOf<Waypoint>(
              Waypoint(
                drivePose.pose2d.translation,
                if (drivetrain.fieldVelocity.magnitude.absoluteValue <
                  0.25.meters.perSecond
                )
                  null
                else heading.inRotation2ds,
                drivePose.rotation.inRotation2ds
              )
            )
        }

        for (vertex in FieldConstants.Zones.farLeftLane.vertices) {
          println("${vertex.x}, ${vertex.y}")
        }
        println("------")
        for (vertex in FieldConstants.Zones.closeLeftLane.vertices) {
          println("${vertex.x}, ${vertex.y}")
        }
      })

    addCommands(
      setupCommand,
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              drivePose.pose2d.translation,
              if (drivetrain.fieldVelocity.magnitude.absoluteValue < 0.25.meters.perSecond)
                null
              else heading.inRotation2ds,
              drivePose.rotation.inRotation2ds
            )
          ) + intermediaryWaypoints
        },
        keepTrapping = true,
        flipForAlliances = false
      ),
      superstructure.groundIntakeConeCommand()
    )
  }
}
