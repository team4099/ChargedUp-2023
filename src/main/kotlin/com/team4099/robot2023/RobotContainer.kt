package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.commands.manipulator.ArmCharacterizationCommand
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
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.InstantCommand
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
      //      vision = Vision(VisionIOSit
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

    // dont home in sim cause theres no output current to check
    manipulator.defaultCommand = InstantCommand({}, manipulator)
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

    ControlBoard.setArmPositionToShelfIntake.whileTrue(manipulator.goToMaxExtensionCommand())
    ControlBoard.extendArm.whileTrue(manipulator.openLoopExtendCommand())
    ControlBoard.retractArm.whileTrue(manipulator.openLoopRetractCommand())

    /*
    ControlBoard.intakeCone.whileTrue(
      manipulator.manipulatorCommand(
        ManipulatorConstants.RollerStates.CONE_IN,
        ManipulatorConstants.ArmStates.MIN_EXTENSION)
    )
    ControlBoard.intakeCube.whileTrue(
      manipulator.manipulatorCommand(
        ManipulatorConstants.RollerStates.CUBE_IN,
        ManipulatorConstants.ArmStates.MIN_EXTENSION)
    )
    ControlBoard.outtakeCone.whileTrue(
      manipulator.manipulatorCommand(
        ManipulatorConstants.RollerStates.CONE_OUT,
        ManipulatorConstants.ArmStates.MIN_EXTENSION
      )
    )
    ControlBoard.outtakeCube.whileTrue(
      manipulator.manipulatorCommand(
        ManipulatorConstants.RollerStates.CUBE_OUT,
        ManipulatorConstants.ArmStates.MIN_EXTENSION
      )
    )
     */
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )"

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun mapTunableCommands() {
    val commandsTab = Shuffleboard.getTab("TunableCommands")
    commandsTab.add(manipulator)
    SendableRegistry.setName(manipulator, "manipulator")
    commandsTab.add("ManipulatorArmCharacterization", ArmCharacterizationCommand(manipulator))
    commandsTab.add(
      "ManipulatorArmTuning",
      manipulator.scoreConeAtHighNodeCommand( // TODO fix
      )
    )
  }
}
