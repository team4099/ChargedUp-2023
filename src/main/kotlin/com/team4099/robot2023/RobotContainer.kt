package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.AutoLevel
import com.team4099.robot2023.commands.drivetrain.GoToAngle
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.Logger
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.VisionIO
import com.team4099.robot2023.subsystems.vision.VisionIOSim
import edu.wpi.first.math.VecBuilder
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOReal)
      vision = Vision(object : VisionIO {})
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      vision = Vision(VisionIOSim)
    }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.robotOriented },
        drivetrain
      )
  }

  val measurementsWithTimestamps
    get() = vision.bestPoses.zip(vision.timestamps).zip(vision.stdevs)

  fun addVisionMeasurement(
    visionPose: Pose2d,
    timestamp: Time,
    visionStdevs: Triple<Double, Double, Double>
  ) {
    drivetrain.swerveDrivePoseEstimator.addVisionMeasurement(
      visionPose.pose2d,
      timestamp.inSeconds,
      VecBuilder.fill(visionStdevs.first, visionStdevs.second, visionStdevs.third)
    )
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
    ControlBoard.autoLevel.whileActiveContinuous(
      GoToAngle(drivetrain).andThen(AutoLevel(drivetrain))
    )
    //
    // ControlBoard.advanceAndClimb.whileActiveOnce(AdvanceClimberCommand().andThen(RunClimbCommand()))
    //        ControlBoard.climbWithoutAdvance.whileActiveOnce(RunClimbCommand())
  }

  fun mapTestControls() {}

  //  fun getAutonomousCommand() =
  //    AutonomousSelector.getCommand(
  //      drivetrain, intake, feeder, shooter, telescopingClimber, pivotClimber
  //    )

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)
}
