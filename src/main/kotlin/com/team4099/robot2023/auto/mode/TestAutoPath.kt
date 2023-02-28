package com.team4099.robot2023.auto.mode

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class TestAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      //      DriveTrajectoryCommand(
      //        drivetrain, trajectoryFromPathPlanner(PathStore.testAutoPath), resetPose = true
      //      )
    )
  }
}
