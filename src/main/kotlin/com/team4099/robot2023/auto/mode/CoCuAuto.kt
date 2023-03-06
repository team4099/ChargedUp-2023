package com.team4099.robot2023.auto.mode

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.pathfollow.Trajectory
import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.commands.drivetrain.DriveTrajectoryCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

class CoCuAuto(val drivetrain: Drivetrain, val superstructure: Superstructure): SequentialCommandGroup(){
  var startingPosToFirstStage: Trajectory = PathStore.rightNodeToFirstStage
  var firstStageToEndPos: Trajectory = PathStore.firstStagetoRightNode

  init{
    addCommands(
      ResetPoseCommand(drivetrain, startingPosToFirstStage.startingPose),
      superstructure.prepscoreCommand(Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HIGH),
      superstructure.scoreConeCommand(),
      WaitCommand(2.0),
      ParallelCommandGroup(
        DriveTrajectoryCommand(drivetrain, startingPosToFirstStage),
        WaitCommand((startingPosToFirstStage.duration - 1.0.seconds).inSeconds).andThen(superstructure.intakeCubeFromGroundCommand())
      ),
      DriveTrajectoryCommand(drivetrain, firstStageToEndPos),
      superstructure.prepscoreCommand(Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HYBRID),
      superstructure.scoreCubeCommand()
    )
  }


  companion object{
    val startingPosX = LoggedTunableValue("Drivetrain/startingPosX", 1.9.meters, Pair({it.inMeters}, {it.meters}))
    val startingPosY = LoggedTunableValue("Drivetrain/startingPosY", 4.97.meters, Pair({it.inMeters}, {it.meters}))
    val startingPosTheta = LoggedTunableValue("Drivetrain/startingPosTheta", 180.0.degrees, Pair({it.inDegrees}, {it.degrees}))

    val endingPosX = LoggedTunableValue("Drivetrain/endingPosX", 1.9.meters, Pair({it.inMeters}, {it.meters}))
    val endingPosY = LoggedTunableValue("Drivetrain/endingPosY", 4.97.meters, Pair({it.inMeters}, {it.meters}))
    val endingPosTheta = LoggedTunableValue("Drivetrain/endingPosTheta", 180.0.degrees, Pair({it.inDegrees}, {it.degrees}))
  }
}
