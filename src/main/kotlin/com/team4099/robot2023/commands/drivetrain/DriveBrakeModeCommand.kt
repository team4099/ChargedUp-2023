package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

class DriveBrakeModeCommand(val drivetrain: Drivetrain) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
      )
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(true) }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(false) }
  }
}
