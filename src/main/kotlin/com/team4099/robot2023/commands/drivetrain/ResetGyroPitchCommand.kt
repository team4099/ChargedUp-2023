package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees

class ResetGyroPitchCommand(val drivetrain: Drivetrain, val toAngle: Angle = 0.0.degrees) :
  CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.gyroIO.zeroGyroPitch(toAngle)
  }

  override fun execute() {
    Logger.getInstance().recordOutput("ActiveCommands/ResetGyroPitchCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}
