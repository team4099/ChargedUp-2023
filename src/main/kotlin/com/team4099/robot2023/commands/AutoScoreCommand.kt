package com.team4099.robot2023.commands

import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.gameboy.objective.isConeNode
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.perSecond

class AutoScoreCommand(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  lateinit var finalPose: Pose2d
  var heading: Angle = 0.0.degrees
  lateinit var gamePiece: GamePiece
  lateinit var nodeTier: NodeTier

  init {

    val setupCommand =
      runOnce({
        drivePose = drivetrain.odometryPose
        finalPose =
          Pose2d(
            1.9.meters,
            FieldConstants.Grids.nodeFirstY +
              FieldConstants.Grids.nodeSeparationY * superstructure.objective.nodeColumn,
            180.degrees
          )
        heading = drivetrain.fieldVelocity.heading
        gamePiece =
          if (superstructure.objective.isConeNode()) Constants.Universal.GamePiece.CONE
          else GamePiece.CUBE
        nodeTier = superstructure.objective.nodeTier
        Logger.getInstance().recordOutput("AutoScore/selectedGamePiece", gamePiece.name)
        Logger.getInstance().recordOutput("AutoScore/selectedNodeTier", nodeTier.name)
      })

    addCommands(
      setupCommand,
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(drivePose.pose2d.translation, if (drivetrain.fieldVelocity.magnitude.absoluteValue < 0.25.meters.perSecond) null else heading.inRotation2ds, drivePose.rotation.inRotation2ds),
            Waypoint(
              finalPose.translation.translation2d, null, finalPose.rotation.inRotation2ds
            )
          )
        },
        keepTrapping = true
      ),
      superstructure.prepScoreCommand({ gamePiece }, { nodeTier }),
      superstructure.score()
    )
  }
}
