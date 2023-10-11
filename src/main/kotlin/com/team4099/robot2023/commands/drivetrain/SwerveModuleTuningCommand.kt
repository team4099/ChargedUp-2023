package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRotation2ds

class SwerveModuleTuningCommand(val drivetrain: Drivetrain, val steeringPosition: () -> Angle) :
  CommandBase() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    for (module in drivetrain.swerveModules) {
      module.setPositionClosedLoop(
        SwerveModuleState(0.0, steeringPosition().inRotation2ds),
        SwerveModuleState(0.0, steeringPosition().inRotation2ds),
        false
      )
    }
  }

  override fun isFinished(): Boolean {
    return false
  }
}
