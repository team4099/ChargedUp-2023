package com.team4099.robot2023

import com.team4099.robot2022.commands.intake.IntakeConeCommand
import com.team4099.robot2022.commands.intake.IntakeCubeCommand
import com.team4099.robot2022.commands.intake.ManipulatorIdleCommand
import com.team4099.robot2022.commands.intake.OuttakeConeCommand
import com.team4099.robot2022.commands.intake.OutttakeCubeCommand
import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIONavx
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import com.team4099.robot2023.subsystems.manipulator.ManipulatorIONeo
import com.team4099.robot2023.subsystems.manipulator.ManipulatorIOSim
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.Logger
import org.team4099.lib.smoothDeadband

object RobotContainer {
  private val drivetrain: Drivetrain
  private val manipulator: Manipulator
  //  private val vision: Vision

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(GyroIONavx, DrivetrainIOReal)
      manipulator = Manipulator(ManipulatorIONeo)
      //      vision = Vision(VisionIOSim)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      manipulator = Manipulator(ManipulatorIOSim)
      //      vision = Vision(VisionIOSim)
    }

    // Set the scheduler to log events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize { command: Command ->
      Logger.getInstance().recordOutput("/ActiveCommands/${command.name}", true)
    }

    CommandScheduler.getInstance().onCommandFinish { command: Command ->
      Logger.getInstance().recordOutput("/ActiveCommands/${command.name}", false)
    }
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

    manipulator.defaultCommand = ManipulatorIdleCommand(manipulator)
  }

  fun zeroSteering() {
    drivetrain.zeroGyroYaw()
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
    //    ControlBoard.extendArm.whileTrue(manipulator.openLoopControl(12.0.volts))
    //    ControlBoard.retractArm.whileTrue(manipulator.openLoopControl(-12.0.volts))
    //
    // ControlBoard.setArmPositionToShelfIntake.whileTrue(manipulator.extendArmPosition(5.inches))
    //    ControlBoard.armCharacterization.whileTrue(ArmCharacterizationCommand(manipulator))
    ControlBoard.intakeCone.whileTrue(IntakeConeCommand(manipulator))
    ControlBoard.intakeCube.whileTrue(IntakeCubeCommand(manipulator))
    ControlBoard.outtakeCone.whileTrue(OuttakeConeCommand(manipulator))
    ControlBoard.outtakeCube.whileTrue(OutttakeCubeCommand(manipulator))
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)
}
