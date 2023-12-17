package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond

class OpenLoopReverseCommand(val drivetrain: Drivetrain) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        0.degrees.perSecond,
        Pair(-5.0.feet.perSecond, 0.0.feet.perSecond),
        fieldOriented = false
      )
  }

  override fun isFinished(): Boolean {
    return false
  }
}
