package com.team4099.robot2022

import com.team4099.lib.smoothDeadband
import com.team4099.robot2022.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2022.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2022.config.ControlBoard
import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.subsystems.drivetrain.Drivetrain
import com.team4099.robot2022.subsystems.drivetrain.DrivetrainIO
import com.team4099.robot2022.subsystems.drivetrain.DrivetrainIOReal

object RobotContainer {
  private val drivetrain: Drivetrain

  init {
    if (Constants.Universal.ROBOT_MODE == Constants.Tuning.RobotType.REAL) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(DrivetrainIOReal)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : DrivetrainIO {})
    }
  }

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.robotOriented },
        drivetrain
      )
    //    PivotClimber.defaultCommand = PivotIdleCommand()
  }

  fun zeroSensors() {
    drivetrain.zeroSensors()
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileActiveOnce(ResetGyroYawCommand(drivetrain))
    //
    // ControlBoard.advanceAndClimb.whileActiveOnce(AdvanceClimberCommand().andThen(RunClimbCommand()))
    //        ControlBoard.climbWithoutAdvance.whileActiveOnce(RunClimbCommand())
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )
}
