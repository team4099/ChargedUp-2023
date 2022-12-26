package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.pathfollow.Trajectory
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase

class LoopPathCommand(val drivetrain: Drivetrain, vararg trajectories: Trajectory) : CommandBase() {
  private val loopedDriveCommands = trajectories.map { DrivePathCommand(drivetrain, it) }
  private var pathIndex = 0

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    pathIndex = 0
    loopedDriveCommands[0].initialize()
  }

  override fun execute() {
    if (loopedDriveCommands[pathIndex].isFinished) {
      pathIndex++
      pathIndex %= loopedDriveCommands.size
      loopedDriveCommands[pathIndex].initialize()
    }
    loopedDriveCommands[pathIndex].execute()
  }
}
