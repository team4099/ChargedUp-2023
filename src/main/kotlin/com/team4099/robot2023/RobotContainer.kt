package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.AutoIntakeCommand
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.elevator.ElevatorIONeo
import com.team4099.robot2023.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2023.subsystems.gameboy.GameBoy
import com.team4099.robot2023.subsystems.gameboy.GameboyIOServer
import com.team4099.robot2023.subsystems.gameboy.objective.isConeNode
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import com.team4099.robot2023.subsystems.groundintake.GroundIntakeIONeo
import com.team4099.robot2023.subsystems.groundintake.GroundIntakeIOSim
import com.team4099.robot2023.subsystems.led.Led
import com.team4099.robot2023.subsystems.led.LedIO
import com.team4099.robot2023.subsystems.led.LedIOSim
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import com.team4099.robot2023.subsystems.manipulator.ManipulatorIONeo
import com.team4099.robot2023.subsystems.manipulator.ManipulatorIOSim
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.camera.CameraIONorthstar
import com.team4099.robot2023.util.driver.Ryan
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import java.util.function.Supplier

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision
  private val superstructure: Superstructure
  private val limelight: LimelightVision

  val rumbleState: Boolean
    get() = superstructure.rumbleState

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      vision =
        Vision(
          //          object: CameraIO {}
          //          CameraIONorthstar("northstar"),
          CameraIONorthstar("northstar_1"),
          CameraIONorthstar("northstar_2"),
          CameraIONorthstar("northstar_3"),
          //        CameraIONorthstar("right"),
          //        CameraIONorthstar("backward")
        )
      superstructure =
        Superstructure(
          Elevator(ElevatorIONeo),
          GroundIntake(GroundIntakeIONeo),
          Manipulator(ManipulatorIONeo),
          Led(object : LedIO {}),
          GameBoy(GameboyIOServer)
        )
      limelight = LimelightVision(object : LimelightVisionIO {})
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      vision =
        Vision(
          CameraIONorthstar("northstar_1"),
          CameraIONorthstar("northstar_2"),
          CameraIONorthstar("northstar_3"),
        )
      superstructure =
        Superstructure(
          Elevator(ElevatorIOSim),
          GroundIntake(GroundIntakeIOSim),
          Manipulator(ManipulatorIOSim),
          Led(LedIOSim),
          GameBoy(GameboyIOServer)
        )
      limelight = LimelightVision(object : LimelightVisionIO {})
    }

    vision.setDataInterfaces({ drivetrain.odometryPose }, { drivetrain.addVisionData(it) })
    drivetrain.elevatorHeightSupplier = Supplier { superstructure.elevatorInputs.elevatorPosition }
    drivetrain.objectiveSupplier = Supplier { superstructure.objective }
    limelight.poseSupplier = { drivetrain.odometryPose }
    limelight.nodeToLookFor = { superstructure.objective }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Ryan(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain
      )
  }

  fun requestSuperstructureIdle() {
    superstructure.currentRequest = Request.SuperstructureRequest.Idle()
  }

  fun zeroArm() {
    superstructure.groundIntakeZeroArm()
  }

  fun regenerateProfiles() {
    superstructure.regenerateProfiles()
  }

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors() {
    drivetrain.zeroSensors()
    superstructure.groundIntakeZeroArm()
  }

  fun zeroAngle(toAngle: Angle) {
    drivetrain.zeroGyroYaw(toAngle)
  }

  fun setSteeringCoastMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(false) }
  }
  fun setSteeringBrakeMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(true) }
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain, toAngle = 180.degrees))
    //    ControlBoard.autoLevel.whileActiveContinuous(
    //      GoToAngle(drivetrain).andThen(AutoLevel(drivetrain))
    //    )

    ControlBoard.setArmCubeHybridPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HYBRID
      )
    )
    ControlBoard.setArmCubeMidPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.MID
      )
    )
    ControlBoard.setArmCubeHighPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CUBE, Constants.Universal.NodeTier.HIGH
      )
    )
    ControlBoard.setArmConeHybridPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HYBRID
      )
    )
    ControlBoard.setArmConeMidPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.MID
      )
    )
    ControlBoard.setArmConeHighPrep.whileTrue(
      superstructure.prepScoreCommand(
        Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.HIGH
      )
    )

    ControlBoard.goBackToIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.scoreOuttake.whileTrue(superstructure.score())
    ControlBoard.singleSubstationIntake.whileTrue(superstructure.singleSubConeCommand())
    ControlBoard.groundIntakeCube.whileTrue(superstructure.groundIntakeCubeCommand())
    ControlBoard.doubleSubstationIntake.whileTrue(superstructure.doubleSubConeCommand())
    ControlBoard.prepScore.whileTrue(
      superstructure.prepScoreCommand(
        {
          if (superstructure.objective.isConeNode()) Constants.Universal.GamePiece.CONE
          else Constants.Universal.GamePiece.CUBE
        },
        { superstructure.objective.nodeTier }
      )
    )

    ControlBoard.groundIntakeCone.whileTrue(superstructure.groundIntakeConeCommand())
    ControlBoard.autoScore.whileTrue(AutoIntakeCommand(drivetrain, superstructure))

    ControlBoard.ejectGamePiece.whileTrue(superstructure.ejectGamePieceCommand())
    //    ControlBoard.dpadDown.whileTrue(PickupFromSubstationCommand(drivetrain, superstructure))

    //    ControlBoard.doubleSubstationIntake.whileTrue(AutoScoreCommand(drivetrain,
    // superstructure))

    //    ControlBoard.doubleSubstationIntake.whileTrue(
    //      PickupFromSubstationCommand(
    //        drivetrain,
    //        superstructure,
    //        Constants.Universal.GamePiece.CONE,
    //        Constants.Universal.Substation.SINGLE_SUBSTATION
    //      )
    //    )
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain, superstructure)

  fun mapTunableCommands() {}
}
