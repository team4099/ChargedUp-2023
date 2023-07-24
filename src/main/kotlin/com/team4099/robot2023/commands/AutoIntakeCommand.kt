package com.team4099.robot2023.commands

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.config.constants.WaypointConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.Constants.Universal.Substation as Substation

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
        drivePose = drivetrain.odometryPose
        heading = drivetrain.fieldVelocity.heading
        currentZone = FieldConstants.determineZone(FieldConstants.Zones.allZones, drivePose)
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

        // Replace last waypoint with the correct waypoint based on operator app
        val finalPose =
          AllianceFlipUtil.apply(
            when (superstructure.objective.substation) {
              Substation.DOUBLE_SUBSTATION_LEFT -> {
                Pose2d(
                  FieldConstants.getTagPose(
                    Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID
                  )!!
                    .toPose2d()
                    .translation +
                    Translation2d(
                      -doubleSubstationXoffset.get(), doubleSubstationYoffset.get()
                    ),
                  0.0.degrees
                )
              }
              Substation.DOUBLE_SUBSTATION_RIGHT -> {
                Pose2d(
                  FieldConstants.getTagPose(
                    Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID
                  )!!
                    .toPose2d()
                    .translation +
                    Translation2d(
                      -doubleSubstationXoffset.get(), -doubleSubstationYoffset.get()
                    ),
                  0.0.degrees
                )
              }
              Substation.SINGLE_SUBSTATION -> {
                Pose2d(
                  FieldConstants.LoadingZone.singleSubstationTranslation +
                    Translation2d(
                      singleSubstationXoffset.get(), -singleSubstationYoffset.get()
                    ),
                  90.0.degrees
                )
              }
              Substation.NONE -> {
                Pose2d(
                  FieldConstants.LoadingZone.singleSubstationTranslation +
                    Translation2d(
                      singleSubstationXoffset.get(), -singleSubstationYoffset.get()
                    ),
                  90.0.degrees
                )
              } // Return single substation by default
            }
          )

        intermediaryWaypoints =
          intermediaryWaypoints.mapIndexed { index, waypoint ->
            waypoint.takeUnless { index == intermediaryWaypoints.size - 1 }
              ?: Waypoint(
                finalPose.pose2d.translation, holonomicRotation = finalPose.pose2d.rotation
              )
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

  companion object {
    val doubleSubstationXoffset =
      LoggedTunableValue(
        "Drivetrain/doubleSubstationXOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
    val doubleSubstationYoffset =
      LoggedTunableValue(
        "Drivetrain/doubleSubstationYOffsetInches",
        0.735.meters,
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationXoffset =
      LoggedTunableValue(
        "Drivetrain/singleSubstationXOffset", 0.0.meters, Pair({ it.inInches }, { it.inches })
      )
    val singleSubstationYoffset =
      LoggedTunableValue(
        "Drivetrain/singleSubstationYOffset", 1.0.meters, Pair({ it.inInches }, { it.inches })
      )
  }
}
