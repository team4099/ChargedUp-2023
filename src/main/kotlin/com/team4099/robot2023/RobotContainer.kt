package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.commands.elevator.ElevatorCharacterizeCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIONavx

import edu.wpi.first.wpilibj.RobotBase
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.elevator.ElevatorIONeo
import com.team4099.robot2023.subsystems.elevator.ElevatorIOSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.Logger
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.volts

object RobotContainer {
  private val drivetrain: Drivetrain
  private val elevator: Elevator
  //  private val vision: Vision

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(GyroIONavx, DrivetrainIOReal)
      elevator = Elevator(ElevatorIONeo)
      //      vision = Vision(VisionIOSim)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      elevator = Elevator(ElevatorIOSim)
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

    //    PivotClimber.defaultCommand = PivotIdleCommand()
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
    ControlBoard.runElevatorToHighNode.whileTrue(
      elevator.raiseElevatorHeight(FieldConstants.Grids.highConeZ)
    )
    ControlBoard.openLoopExtend.whileTrue(elevator.openLoopControl(12.volts))
    ControlBoard.openLoopRetract.whileTrue(elevator.openLoopControl(-12.volts))
    ControlBoard.characterizeElevator.whileTrue(ElevatorCharacterizeCommand(elevator))
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)
}
