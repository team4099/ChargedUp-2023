package com.team4099.robot2023.commands.drivetrain

// class LoopPathCommand(val drivetrain: Drivetrain, vararg trajectories: Trajectory) :
// CommandBase() {
//  private val loopedDriveCommands = trajectories.map { DrivePathCommand(drivetrain, it) }
//  private var pathIndex = 0
//
//  init {
//    addRequirements(drivetrain)
//  }
//
//  override fun initialize() {
//    pathIndex = 0
//    loopedDriveCommands[0].initialize()
//  }
//
//  override fun execute() {
//    if (loopedDriveCommands[pathIndex].isFinished) {
//      pathIndex++
//      pathIndex %= loopedDriveCommands.size
//      loopedDriveCommands[pathIndex].initialize()
//    }
//    loopedDriveCommands[pathIndex].execute()
//  }
// }
