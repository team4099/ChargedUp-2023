package com.team4099.robot2023.auto.mode

import com.team4099.robot2023.commands.drivetrain.OpenLoopReverseCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class PreloadOpenLoopChargeStationBalance(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : SequentialCommandGroup() {

  init {
    addCommands(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HIGH
      ),
      superstructure.score(),
      ParallelRaceGroup(OpenLoopReverseCommand(drivetrain), WaitCommand(1.3)),
    )
  }
}
