package com.team4099.robot2023.auto.mode

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class LeftAutoTest(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    val startingPosX = drivetrain.odometryPose.x
    val startingPosY = drivetrain.odometryPose.y

    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              Translation2d(startingPosX, startingPosY).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
            Waypoint(
              Translation2d(startingPosX, startingPosY-1.0.meters).translation2d,
              null, 0.0.degrees.inRotation2ds
            ),
          )
        },
        resetPose = true
      )
    )
  }
}
