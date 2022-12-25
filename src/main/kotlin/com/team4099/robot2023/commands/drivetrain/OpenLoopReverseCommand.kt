package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.units.base.feet
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase

class OpenLoopReverseCommand(val drivetrain: Drivetrain) : CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.setOpenLoop(
      0.degrees.perSecond, Pair(0.0.feet.perSecond, 2.5.feet.perSecond), fieldOriented = false
    )
  }

  override fun isFinished(): Boolean {
    return false
  }
}
