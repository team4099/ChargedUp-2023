package com.team4099.robot2022.commands.drivetrain

import com.team4099.lib.geometry.Pose
import com.team4099.robot2022.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger

class ResetPoseCommand(val drivetrain: Drivetrain, val pose: Pose) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.pose = pose
    Logger.getInstance().recordOutput("ActiveCommands/ResetPoseCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}
