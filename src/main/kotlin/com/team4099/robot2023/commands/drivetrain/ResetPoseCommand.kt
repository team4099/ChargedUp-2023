package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d

class ResetPoseCommand(val drivetrain: Drivetrain, val pose: Pose2d) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.odometryPose = pose
    Logger.getInstance().recordOutput("ActiveCommands/ResetPoseCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}
