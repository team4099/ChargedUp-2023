package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class TestAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              Translation2d(0.0.meters, 0.0.meters).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
            Waypoint(
              Translation2d(2.0.meters, 0.0.meters).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
            Waypoint(
              Translation2d(0.0.meters, 1.0.meters).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
          )
        },
        resetPose = true
      )
    )
  }
}
