package com.team4099.robot2023.auto.mode

import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.pathfollow.trajectoryFromPathPlanner

class TestAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand(
        drivetrain, trajectoryFromPathPlanner(PathStore.testAutoPath), resetPose = true
      )
    )
  }
}
