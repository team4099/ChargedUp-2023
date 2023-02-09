package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.commands.elevator.GroundIntakeCharacterizeCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIONavx
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import com.team4099.robot2023.subsystems.groundintake.GroundIntakeIONeo
import com.team4099.robot2023.subsystems.groundintake.GroundIntakeIOSim
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.degrees

object RobotContainer {
  private val drivetrain: Drivetrain
  //  private val vision: Vision

  private val groundIntake: GroundIntake

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(GyroIONavx, DrivetrainIOReal)
      //      vision = Vision(VisionIOSim)

      groundIntake = GroundIntake(GroundIntakeIONeo)
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      //      vision = Vision(VisionIOSim)

      groundIntake = GroundIntake(GroundIntakeIOSim)
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

    groundIntake.defaultCommand = groundIntake.holdArmPosition()
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

    ControlBoard.extendIntake.whileTrue(
      groundIntake.groundIntakeDeployCommand(GroundIntakeConstants.ArmStates.INTAKE)
    )
    ControlBoard.retractIntake.whileTrue(
      groundIntake.groundIntakeDeployCommand(GroundIntakeConstants.ArmStates.STOWED)
    )
    ControlBoard.characterizeIntake.whileTrue(
      groundIntake.rotateGroundIntakeToAngle(5.0.degrees.asSupplier)
    )

    ControlBoard.setArmCommand.whileTrue(
      groundIntake.rotateGroundIntakeToAngle(45.degrees.asSupplier)
    )
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun mapTunableCommands() {
    val commandsTab = Shuffleboard.getTab("TunableCommands")
    commandsTab.add(groundIntake)
    SendableRegistry.setName(groundIntake, "groundIntake")
    commandsTab.add(
      "GroundIntakeArmCharacterization", GroundIntakeCharacterizeCommand(groundIntake)
    )
    commandsTab.add(
      "GroundIntakeArmTuning",
      groundIntake.rotateGroundIntakeToAngle(
        GroundIntakeConstants.ArmStates.TUNABLE_STATE.position.asSupplier
      )
    )
  }
}
