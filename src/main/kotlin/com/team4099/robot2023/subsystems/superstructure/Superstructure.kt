package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.commands.elevator.ElevatorKsCharacterizeCommand
import com.team4099.robot2023.commands.elevator.GroundIntakeCharacterizeCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.LedConstants.LEDMode
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.gameboy.GameBoy
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import com.team4099.robot2023.subsystems.led.Led
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import java.util.function.Supplier
import com.team4099.robot2023.subsystems.superstructure.Request.SuperstructureRequest as SuperstructureRequest

class Superstructure(
  private val elevator: Elevator,
  private val groundIntake: GroundIntake,
  private val manipulator: Manipulator,
  private val led: Led,
  private val gameboy: GameBoy
) : SubsystemBase() {

  var currentRequest: SuperstructureRequest = SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is SuperstructureRequest.GroundIntakeCone -> usingGamePiece = GamePiece.CONE
        is SuperstructureRequest.GroundIntakeCube -> usingGamePiece = GamePiece.CUBE
        is SuperstructureRequest.DoubleSubstationIntake -> {}
        is SuperstructureRequest.SingleSubstationIntake -> usingGamePiece = value.gamePiece
        is SuperstructureRequest.PrepScore -> {
          usingGamePiece = value.gamePiece
          nodeTier = value.nodeTier
        }
        is SuperstructureRequest.DoubleSubstationIntakePrep -> usingGamePiece = value.gamePiece
        is SuperstructureRequest.SingleSubstationIntakePrep -> usingGamePiece = value.gamePiece
        is SuperstructureRequest.Score -> {}
        else -> {
          usingGamePiece = GamePiece.NONE
          nodeTier = NodeTier.NONE
        }
      }
      field = value
    }

  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var usingGamePiece: GamePiece = GamePiece.NONE

  var theoreticalGamePiece: GamePiece = GamePiece.NONE

  var nodeTier: NodeTier = NodeTier.NONE

  var lastTransitionTime = Clock.fpgaTime

  val elevatorInputs = elevator.inputs

  val groundIntakeInputs = groundIntake.inputs

  val manipulatorInputs = manipulator.inputs

  val objective = gameboy.inputs.objective

  var isAtRequestedState: Boolean = false

  var checkAtRequestedStateNextLoopCycle = false

  var canMoveSafely = false

  var scoringConeWithoutLoweringGroundIntake = false

  val rumbleState: Boolean
    get() {
      return manipulator.rumbleTrigger
    }

  override fun periodic() {
    val ledLoopStartTime = Clock.realTimestamp
    led.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/LedLoopTimeMS",
        (Clock.realTimestamp - ledLoopStartTime).inMilliseconds
      )

    val gameboyLoopStartTime = Clock.realTimestamp
    gameboy.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/GameBoyLoopTimeMS",
        (Clock.realTimestamp - gameboyLoopStartTime).inMilliseconds
      )

    val elevatorLoopStartTime = Clock.realTimestamp
    elevator.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/ElevatorLoopTimeMS",
        (Clock.realTimestamp - elevatorLoopStartTime).inMilliseconds
      )

    val groundIntakeLoopStartTime = Clock.realTimestamp
    groundIntake.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/GroundIntakeLoopTimeMS",
        (Clock.realTimestamp - groundIntakeLoopStartTime).inMilliseconds
      )

    val manipulatorLoopStartTime = Clock.realTimestamp
    manipulator.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/ManipulatorLoopTimeMS",
        (Clock.realTimestamp - manipulatorLoopStartTime).inMilliseconds
      )

    val superstructureStateMachineStartTime = Clock.realTimestamp

    Logger.getInstance()
      .recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    Logger.getInstance().recordOutput("Superstructure/currentState", currentState.name)
    Logger.getInstance().recordOutput("Superstructure/usingGamePiece", usingGamePiece.name)
    Logger.getInstance()
      .recordOutput("Superstructure/holdingGamePiece", manipulator.holdingGamePiece.name)
    Logger.getInstance().recordOutput("Superstructure/nodeTier", nodeTier.name)
    Logger.getInstance()
      .recordOutput("Superstructure/lastTransitionTime", lastTransitionTime.inSeconds)
    Logger.getInstance().recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)
    Logger.getInstance()
      .recordOutput("Superstructure/theoreticalGamePiece", theoreticalGamePiece.name)
    Logger.getInstance().recordOutput("Superstructure/canMoveSafely", canMoveSafely)

    Logger.getInstance()
      .recordOutput(
        "SimulatedMechanisms",
        Pose3d(
          0.1016.meters,
          0.0.meters,
          0.211550.meters,
          Rotation3d(0.0.degrees, -groundIntake.inputs.armPosition, 0.0.degrees)
        )
          .pose3d,
        if (elevator.inputs.elevatorPosition >= ElevatorConstants.FIRST_STAGE_HEIGHT)
          Pose3d()
            .transformBy(
              Transform3d(
                Translation3d(
                  0.0.inches,
                  0.0.inches,
                  elevator.inputs.elevatorPosition -
                    ElevatorConstants.FIRST_STAGE_HEIGHT
                )
                  .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
                Rotation3d()
              )
            )
            .pose3d
        else
          Pose3d()
            .transformBy(
              Transform3d(
                Translation3d(0.0.inches, 0.0.inches, 0.0.inches)
                  .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
                Rotation3d()
              )
            )
            .pose3d,
        Pose3d()
          .transformBy(
            Transform3d(
              Translation3d(0.0.inches, 0.0.inches, elevator.inputs.elevatorPosition)
                .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
              Rotation3d()
            )
          )
          .pose3d,
        Pose3d(
          -0.15.meters + manipulator.inputs.armPosition,
          0.05.meters,
          0.5825.meters,
          Rotation3d()
        )
          .transformBy(
            Transform3d(
              Translation3d(0.0.inches, 0.0.inches, elevator.inputs.elevatorPosition)
                .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
              Rotation3d()
            )
          )
          .pose3d
      )

    var nextState = currentState
    when (currentState) {
      SuperstructureStates.UNINITIALIZED -> {
        // Outputs

        // Transition
        nextState = SuperstructureStates.HOME_PREP
      }
      SuperstructureStates.IDLE -> {
        if (DriverStation.isAutonomous()) {
          led.state = LEDMode.AUTO
        } else if (DriverStation.isDisabled()) {
          led.state = LEDMode.IDLE
        } else {
          when (manipulator.holdingGamePiece) {
            GamePiece.CONE -> led.state = LEDMode.CONE
            GamePiece.CUBE -> led.state = LEDMode.CUBE
            else -> LEDMode.TELEOP
          }
        }

        // Outputs
        val rollerVoltage =
          when (theoreticalGamePiece) {
            GamePiece.NONE -> {
              0.0.volts
            }
            GamePiece.CONE -> {
              Manipulator.TunableManipulatorStates.coneIdleVoltage.get()
            }
            GamePiece.CUBE -> {
              Manipulator.TunableManipulatorStates.cubeIdleVoltage.get()
            }
          }

        if (elevator.isStowed &&
          elevator.isAtTargetedPosition &&
          manipulator.isStowed &&
          manipulator.isAtTargetedPosition
        ) {
          if (!DriverStation.isAutonomous()) {
            groundIntake.currentRequest =
              Request.GroundIntakeRequest.TargetingPosition(
                GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(),
                GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
              )
          } else {
            groundIntake.currentRequest =
              Request.GroundIntakeRequest.TargetingPosition(
                GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
                GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
              )
          }
        } else {
          groundIntake.currentRequest =
            Request.GroundIntakeRequest.TargetingPosition(
              GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
              GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
            )
        }
        if (groundIntake.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.minExtension.get(), rollerVoltage
            )

          if (manipulator.isAtTargetedPosition) {
            elevator.currentRequest =
              Request.ElevatorRequest.TargetingPosition(
                Elevator.TunableElevatorHeights.minPosition.get()
              )
          }
        } else {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              manipulator.armPositionTarget, rollerVoltage
            )
        }

        // Transition
        nextState =
          when (currentRequest) {
            is SuperstructureRequest.DoubleSubstationIntake ->
              SuperstructureStates.DOUBLE_SUBSTATION_INTAKE
            is SuperstructureRequest.DoubleSubstationIntakePrep ->
              SuperstructureStates.DOUBLE_SUBSTATION_INTAKE_PREP
            is SuperstructureRequest.GroundIntakeCone ->
              SuperstructureStates.GROUND_INTAKE_CONE_PREP
            is SuperstructureRequest.GroundIntakeCube ->
              SuperstructureStates.GROUND_INTAKE_CUBE_PREP
            is SuperstructureRequest.Home -> SuperstructureStates.HOME_PREP
            is SuperstructureRequest.Idle -> SuperstructureStates.IDLE
            is SuperstructureRequest.SingleSubstationIntakePrep ->
              SuperstructureStates.SINGLE_SUBSTATION_INTAKE_PREP
            is SuperstructureRequest.EjectGamePiece -> SuperstructureStates.EJECT_GAME_PIECE
            is SuperstructureRequest.PrepScore -> {
              when (usingGamePiece) {
                GamePiece.CONE -> SuperstructureStates.SCORE_PREP
                GamePiece.CUBE -> SuperstructureStates.SCORE_PREP
                else -> {
                  currentRequest = SuperstructureRequest.Idle()
                  SuperstructureStates.IDLE
                }
              }
            }
            is SuperstructureRequest.Tuning -> SuperstructureStates.TUNING
            else -> currentState
          }
      }
      SuperstructureStates.HOME_PREP -> {

        led.state = LEDMode.MOVEMENT

        // Outputs
        //        groundIntake.currentRequest = Request.GroundIntakeRequest.OpenLoop(-10.volts,
        // 0.0.volts)
        //
        //        if ((Clock.fpgaTime - lastTransitionTime) >=
        //          0.5.seconds){
        groundIntake.currentRequest = Request.GroundIntakeRequest.ZeroArm()
        //        }

        if (groundIntake.isZeroed) {
          groundIntake.currentRequest =
            Request.GroundIntakeRequest.TargetingPosition(
              GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
              GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
            )
        }

        // Transition
        if (groundIntake.isAtTargetedPosition && groundIntake.isZeroed) {
          nextState = SuperstructureStates.HOME
        }
      }
      SuperstructureStates.HOME -> {

        led.state = LEDMode.MOVEMENT

        // Outputs
        elevator.currentRequest = Request.ElevatorRequest.Home()
        manipulator.currentRequest = Request.ManipulatorRequest.Home()

        // Transition
        if (elevator.isHomed && manipulator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE_PREP -> {
        led.state = LEDMode.INTAKE

        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.intakeAngle.get(),
            GroundIntake.TunableGroundIntakeStates.intakeVoltage.get(),
          )
        if (groundIntake.isAtTargetedPosition) {
          val rollerCommandedVoltage =
            when (usingGamePiece) {
              GamePiece.CONE -> ManipulatorConstants.CONE_IDLE
              GamePiece.CUBE -> ManipulatorConstants.CUBE_IDLE
              else -> 0.0.volts
            }

          elevator.currentRequest =
            Request.ElevatorRequest.TargetingPosition(
              Elevator.TunableElevatorHeights.groundIntakeCubeHeight.get()
            )

          if (elevator.isAtTargetedPosition) {
            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                Manipulator.TunableManipulatorStates.groundIntakeCubeExtension.get(),
                rollerCommandedVoltage
              )
          }
        }

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          elevator.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCube) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.OpenLoop(
            -5.volts, GroundIntake.TunableGroundIntakeStates.intakeVoltage.get()
          )

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.groundIntakeCubeExtension.get(),
            Manipulator.TunableManipulatorStates.cubeInVoltage.get()
          )

        // Transition
        if (manipulator.isAtTargetedPosition && manipulator.hasCube) {
          theoreticalGamePiece = GamePiece.CUBE
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCube) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP -> {
        // Goes immediately to IDLE
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )

        if (groundIntake.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
        }

        // Transition
        currentRequest = SuperstructureRequest.Idle()
        nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.GROUND_INTAKE_CONE_PREP -> {

        led.state = LEDMode.INTAKE

        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(), 0.0.volts
          )

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            manipulator.inputs.armPosition,
            Manipulator.TunableManipulatorStates.coneInVoltage.get()
          )

        if (groundIntake.isAtTargetedPosition || groundIntake.canContinueSafely) {
          elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(2.5.inches)
          if (elevator.isAtTargetedPosition) {
            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                7.0.inches, Manipulator.TunableManipulatorStates.coneInVoltage.get()
              )
          }
        }

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.GROUND_INTAKE_CONE
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCone) {
          nextState = SuperstructureStates.GROUND_INTAKE_CONE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CONE -> {
        // Outputs
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            7.0.inches, Manipulator.TunableManipulatorStates.coneInVoltage.get()
          )

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition &&
          manipulator.hasCone
        ) {
          theoreticalGamePiece = Constants.Universal.GamePiece.CONE
          nextState = SuperstructureStates.GROUND_INTAKE_CONE_CLEANUP
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCone) {
          nextState = SuperstructureStates.GROUND_INTAKE_CONE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CONE_CLEANUP -> {
        // Outputs
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.minExtension.get(),
            Manipulator.TunableManipulatorStates.coneIdleVoltage.get()
          )

        if (manipulator.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.TargetingPosition(
              Elevator.TunableElevatorHeights.minPosition.get()
            )
        }

        // Transition
        if (manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.intakeTime.get()
        ) {
          currentRequest = SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.DOUBLE_SUBSTATION_INTAKE_PREP -> {

        led.state = LEDMode.DOUBLE_SUBSTATION

        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.canContinueSafely || groundIntake.isAtTargetedPosition) {
          val offset =
            when (usingGamePiece) {
              GamePiece.CUBE -> Elevator.TunableElevatorHeights.shelfIntakeCubeOffset.get()
              GamePiece.CONE -> Elevator.TunableElevatorHeights.shelfIntakeConeOffset.get()
              else -> 0.0.inches
            }
          elevator.currentRequest =
            Request.ElevatorRequest.TargetingPosition(
              Elevator.TunableElevatorHeights.doubleSubstationHeight.get() + offset
            )

          if (elevator.isAtTargetedPosition) {
            val rollerCommandedVoltage =
              when (usingGamePiece) {
                GamePiece.CONE -> ManipulatorConstants.CONE_IN
                GamePiece.CUBE -> ManipulatorConstants.CUBE_IN
                else -> -1.0.volts
              }

            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                Manipulator.TunableManipulatorStates.doubleSubstationIntakeShelfExtension.get(),
                rollerCommandedVoltage
              )
          }
        }

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.DOUBLE_SUBSTATION_INTAKE
        } else if (currentRequest !is SuperstructureRequest.DoubleSubstationIntake &&
          currentRequest !is SuperstructureRequest.DoubleSubstationIntakePrep
        ) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.DOUBLE_SUBSTATION_INTAKE -> {

        led.state = LEDMode.INTAKE

        // Outputs
        val gripperVoltage =
          when (usingGamePiece) {
            GamePiece.CUBE -> Manipulator.TunableManipulatorStates.cubeInVoltage.get()
            GamePiece.CONE -> Manipulator.TunableManipulatorStates.coneInVoltage.get()
            else -> ManipulatorConstants.IDLE_VOLTAGE
          }
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.doubleSubstationIntakeShelfExtension.get(),
            gripperVoltage
          )

        // Transition
        if ((
          manipulator.isAtTargetedPosition &&
            (
              (manipulator.hasCone && usingGamePiece == GamePiece.CONE) ||
                (manipulator.hasCube && usingGamePiece == GamePiece.CUBE)
              )
          ) ||
          currentRequest is SuperstructureRequest.Idle
        ) {
          if (manipulator.hasCone && usingGamePiece == GamePiece.CONE) {
            theoreticalGamePiece = Constants.Universal.GamePiece.CONE
          }
          nextState = SuperstructureStates.DOUBLE_SUBSTATION_CLEANUP
        }
      }
      SuperstructureStates.SINGLE_SUBSTATION_INTAKE_PREP -> {

        led.state = LEDMode.SINGLE_SUBSTATION

        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition || groundIntake.canContinueSafely) {
          val offset =
            when (usingGamePiece) {
              GamePiece.CUBE -> Elevator.TunableElevatorHeights.singleSubstationCubeOffset.get()
              GamePiece.CONE -> Elevator.TunableElevatorHeights.singleSubstationConeOffset.get()
              else -> 0.0.inches
            }
          elevator.currentRequest =
            Request.ElevatorRequest.TargetingPosition(
              Elevator.TunableElevatorHeights.singleSubstationHeight.get() + offset
            )

          if (elevator.isAtTargetedPosition || elevator.canContinueSafely) {
            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
                ManipulatorConstants.IDLE_VOLTAGE
              )
          }
        }
        if (groundIntake.isAtTargetedPosition && elevator.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
              ManipulatorConstants.IDLE_VOLTAGE
            )
        }

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition
        ) {
          nextState =
            when (usingGamePiece) {
              GamePiece.CUBE -> SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CUBE
              GamePiece.CONE -> SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CONE
              else -> SuperstructureStates.IDLE
            }
        }
      }
      SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CUBE -> {

        led.state = LEDMode.INTAKE

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
            Manipulator.TunableManipulatorStates.cubeInVoltage.get()
          )
        // Outputs

        // Transition
        if (manipulator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.intakeTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        }
      }
      SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CONE -> {

        led.state = LEDMode.INTAKE

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
            Manipulator.TunableManipulatorStates.coneInVoltage.get()
          )
        // Outputs

        // Transition
        if ((
          manipulator.isAtTargetedPosition &&
            (
              (manipulator.hasCone && usingGamePiece == GamePiece.CONE) ||
                (manipulator.hasCube && usingGamePiece == GamePiece.CUBE)
              )
          ) ||
          currentRequest is SuperstructureRequest.Idle
        ) {
          if (manipulator.hasCone && usingGamePiece == GamePiece.CONE) {
            theoreticalGamePiece = Constants.Universal.GamePiece.CONE
          }
          nextState = SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CLEANUP
        }
      }
      SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CLEANUP -> {
        led.state = LEDMode.MOVEMENT

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.minExtension.get(),
            ManipulatorConstants.CONE_IDLE
          )

        if (manipulator.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        }
      }
      SuperstructureStates.SCORE_PREP -> {
        led.state = LEDMode.SCORE

        val rollerVoltage =
          when (usingGamePiece) {
            GamePiece.CUBE ->
              when (nodeTier) {
                NodeTier.HYBRID -> GroundIntake.TunableGroundIntakeStates.outtakeVoltage.get()
                else -> 0.0.volts
              }
            else -> 0.0.volts
          }

        if (usingGamePiece == Constants.Universal.GamePiece.CONE &&
          nodeTier == Constants.Universal.NodeTier.HYBRID
        ) {
          if (elevatorInputs.elevatorPosition >=
            10.inches
          ) { // elevator be high up, faster to just score from above
            scoringConeWithoutLoweringGroundIntake = false
            groundIntake.currentRequest =
              Request.GroundIntakeRequest.TargetingPosition(
                GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(), rollerVoltage
              )
          } else {
            scoringConeWithoutLoweringGroundIntake = true
            groundIntake.currentRequest =
              Request.GroundIntakeRequest.TargetingPosition(
                GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(), rollerVoltage
              )
          }
        } else {
          if (scoringConeWithoutLoweringGroundIntake) { // previously scored without lowering ground
            // intake so we need to retract manipulator
            // and lower elevator
            val rollerCommandedVoltage =
              when (usingGamePiece) {
                GamePiece.CONE -> ManipulatorConstants.CONE_IDLE
                GamePiece.CUBE -> ManipulatorConstants.CUBE_IDLE
                else -> -1.0.volts
              }

            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                Manipulator.TunableManipulatorStates.minExtension.get(), rollerCommandedVoltage
              )
            if (manipulator.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.TargetingPosition(
                  Elevator.TunableElevatorHeights.minPosition.get()
                )
              if (elevator.isAtTargetedPosition) {
                groundIntake.currentRequest =
                  Request.GroundIntakeRequest.TargetingPosition(
                    GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(), rollerVoltage
                  )
                scoringConeWithoutLoweringGroundIntake = false
              }
            }
          } else {
            groundIntake.currentRequest =
              Request.GroundIntakeRequest.TargetingPosition(
                GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(), rollerVoltage
              )
          }
        }

        if (manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition &&
          (
            groundIntake.isAtTargetedPosition ||
              groundIntake
                .canContinueSafely
            )
        ) { // because manipulator and elevator will move while we're
          // switching between different cone tiers
          val rollerCommandedVoltage =
            when (usingGamePiece) {
              GamePiece.CONE -> ManipulatorConstants.CONE_IDLE
              GamePiece.CUBE -> ManipulatorConstants.CUBE_IDLE
              else -> -1.0.volts
            }

          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.minExtension.get(), rollerCommandedVoltage
            )

          val scoreHeight =
            when (nodeTier) {
              NodeTier.HYBRID -> {
                when (usingGamePiece) {
                  GamePiece.CUBE -> Elevator.TunableElevatorHeights.groundIntakeCubeHeight.get()
                  GamePiece.CONE ->
                    if (scoringConeWithoutLoweringGroundIntake) {
                      Elevator.TunableElevatorHeights.coneDropPosition.get() + 3.inches
                    } else {
                      Elevator.TunableElevatorHeights.hybridHeight.get() +
                        Elevator.TunableElevatorHeights.coneDropPosition.get()
                    }
                  else -> 0.0.inches
                }
              }
              NodeTier.MID -> {
                when (usingGamePiece) {
                  GamePiece.CUBE ->
                    Elevator.TunableElevatorHeights.midCubeHeight.get() +
                      Elevator.TunableElevatorHeights.cubeDropPosition.get()
                  GamePiece.CONE ->
                    Elevator.TunableElevatorHeights.midConeHeight.get() +
                      Elevator.TunableElevatorHeights.coneDropPosition.get()
                  else -> 0.0.inches
                }
              }
              NodeTier.HIGH -> {
                when (usingGamePiece) {
                  GamePiece.CUBE ->
                    Elevator.TunableElevatorHeights.highCubeHeight.get() +
                      Elevator.TunableElevatorHeights.cubeDropPosition.get()
                  GamePiece.CONE ->
                    Elevator.TunableElevatorHeights.highConeHeight.get() +
                      Elevator.TunableElevatorHeights.coneDropPosition.get()
                  else -> 0.0.inches
                }
              }
              else -> 0.0.inches
            }

          elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(scoreHeight)

          if (elevator.isAtTargetedPosition || elevator.canContinueSafely) {

            val extension =
              when (nodeTier) {
                NodeTier.HYBRID -> {
                  when (usingGamePiece) {
                    GamePiece.CONE ->
                      if (scoringConeWithoutLoweringGroundIntake) {
                        Manipulator.TunableManipulatorStates.lowConeScoreExtension.get()
                      } else {
                        Manipulator.TunableManipulatorStates.lowConeScoreExtension.get() -
                          5.inches
                      }
                    GamePiece.CUBE ->
                      Manipulator.TunableManipulatorStates.lowCubeScoreExtension.get()
                    else -> 0.0.inches
                  }
                }
                NodeTier.MID -> {
                  when (usingGamePiece) {
                    GamePiece.CONE ->
                      Manipulator.TunableManipulatorStates.midConeScoreExtension.get()
                    GamePiece.CUBE ->
                      Manipulator.TunableManipulatorStates.midCubeScoreExtension.get()
                    else -> 0.0.inches
                  }
                }
                NodeTier.HIGH -> {
                  when (usingGamePiece) {
                    GamePiece.CONE ->
                      Manipulator.TunableManipulatorStates.highConeScoreExtension.get()
                    GamePiece.CUBE ->
                      Manipulator.TunableManipulatorStates.highCubeScoreExtension.get()
                    else -> 0.0.inches
                  }
                }
                else -> 0.0.inches
              }

            when (usingGamePiece) {
              Constants.Universal.GamePiece.NONE ->
                manipulator.currentRequest =
                  Request.ManipulatorRequest.TargetingPosition(
                    extension, ManipulatorConstants.IDLE_VOLTAGE
                  )
              Constants.Universal.GamePiece.CONE ->
                manipulator.currentRequest =
                  Request.ManipulatorRequest.TargetingPosition(
                    extension, ManipulatorConstants.CONE_IDLE
                  )
              Constants.Universal.GamePiece.CUBE ->
                manipulator.currentRequest =
                  Request.ManipulatorRequest.TargetingPosition(
                    extension, ManipulatorConstants.CUBE_IDLE
                  )
            }
          }
        }

        // Transition
        if (currentRequest is SuperstructureRequest.Score &&
          groundIntake.isAtTargetedPosition &&
          elevator.isAtTargetedPosition
        ) {
          nextState =
            when (usingGamePiece) {
              GamePiece.CUBE -> SuperstructureStates.SCORE_CUBE
              GamePiece.CONE -> SuperstructureStates.SCORE_CONE
              else -> SuperstructureStates.IDLE
            }
        } else if (currentRequest is SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_CUBE -> {
        led.state = LEDMode.OUTTAKE
        // Outputs
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            manipulator.armPositionTarget,
            Manipulator.TunableManipulatorStates.cubeOutVoltage.get()
          )

        // Transition
        if (manipulator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.spitTime.get()
        ) {
          theoreticalGamePiece = Constants.Universal.GamePiece.NONE
          nextState = SuperstructureStates.SCORE_CLEANUP
        } else if (currentRequest !is SuperstructureRequest.Score) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_CONE -> {
        led.state = LEDMode.OUTTAKE
        // Outputs
        val dropToPosition = elevator.elevatorPositionTarget

        elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(dropToPosition)
        if (elevator.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              manipulator.armPositionTarget,
              Manipulator.TunableManipulatorStates.coneOutVoltage.get()
            )
        }

        // Transition
        if (elevator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.spitTimeCone.get()
        ) {
          theoreticalGamePiece = GamePiece.NONE
          nextState = SuperstructureStates.SCORE_CLEANUP
        } else if (currentRequest !is SuperstructureRequest.Score) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_CLEANUP -> {
        led.state = LEDMode.MOVEMENT
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.minExtension.get(),
            ManipulatorConstants.IDLE_VOLTAGE
          )
        /*
              if (nodeTier == Constants.Universal.NodeTier.HYBRID) {
                if (manipulator.isAtTargetedPosition) {
                  elevator.currentRequest =
                    Request.ElevatorRequest.TargetingPosition(
                      Elevator.TunableElevatorHeights.minPosition.get()
                    )
                }
              }
        */
        if (manipulator.isAtTargetedPosition && elevator.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        } else if (currentRequest is SuperstructureRequest.PrepScore) {
          nextState = SuperstructureStates.SCORE_PREP
        }
      }
      SuperstructureStates.EJECT_GAME_PIECE -> {

        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.outtakeAngle.get(),
            GroundIntake.TunableGroundIntakeStates.outtakeVoltage.get()
          )

        if (groundIntake.isAtTargetedPosition) {

          elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(23.0.inches)

          if (elevator.canContinueSafely || elevator.isAtTargetedPosition) {
            manipulator.currentRequest =
              Request.ManipulatorRequest.TargetingPosition(
                Manipulator.TunableManipulatorStates.minExtension.get(),
                Manipulator.TunableManipulatorStates.cubeOutVoltage.get()
              )
          }
        }

        if (currentRequest is SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        }
      }
      SuperstructureStates.DOUBLE_SUBSTATION_CLEANUP -> {
        led.state = LEDMode.MOVEMENT

        val rollerCommandedVoltage =
          when (usingGamePiece) {
            GamePiece.CONE -> ManipulatorConstants.CONE_IN
            GamePiece.CUBE -> ManipulatorConstants.CUBE_IDLE
            else -> -1.0.volts
          }

        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.minExtension.get(), rollerCommandedVoltage
          )

        if (manipulator.isAtTargetedPosition) {
          if (usingGamePiece == GamePiece.CONE) {
            led.state = LEDMode.CONE
          } else {
            led.state = LEDMode.CUBE
          }

          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        }
      }
      SuperstructureStates.TUNING -> {

        if (currentRequest is SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
    }

    if (nextState != currentState) {
      lastTransitionTime = Clock.fpgaTime
      checkAtRequestedStateNextLoopCycle = true
    }

    currentState = nextState

    if (!(checkAtRequestedStateNextLoopCycle)) {
      isAtRequestedState =
        elevator.isAtTargetedPosition &&
        manipulator.isAtTargetedPosition &&
        groundIntake.isAtTargetedPosition

      canMoveSafely = elevator.canContinueSafely && groundIntake.canContinueSafely
    } else {
      checkAtRequestedStateNextLoopCycle = false
    }

    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/SuperstructureStateMachineLoopTimeMS",
        (Clock.realTimestamp - superstructureStateMachineStartTime).inMilliseconds
      )
  }

  // Superstructure Commands

  fun ejectGamePieceCommand(): CommandBase {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.EjectGamePiece() }.until {
        isAtRequestedState && currentState == SuperstructureStates.EJECT_GAME_PIECE
      }

    returnCommand.name = "EjectGamePieceCommand"
    return returnCommand
  }

  fun requestIdleCommand(): CommandBase {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.Idle() }.until {
        currentState == SuperstructureStates.IDLE
      }

    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun groundIntakeCubeCommand(): CommandBase {
    val returnCommand = runOnce { currentRequest = SuperstructureRequest.GroundIntakeCube() }

    returnCommand.name = "GroundIntakeCubeCommand"
    return returnCommand
  }

  fun prepScoreCommand(
    gamePieceSupplier: Supplier<GamePiece>,
    nodeTierSupplier: Supplier<NodeTier>
  ): CommandBase {
    val returnCommand =
      runOnce {
        currentRequest =
          SuperstructureRequest.PrepScore(gamePieceSupplier.get(), nodeTierSupplier.get())
      }
        .andThen(
          WaitUntilCommand {
            isAtRequestedState && currentState == SuperstructureStates.SCORE_PREP
          }
        )

    return returnCommand
  }

  fun prepScoreCommand(gamePiece: GamePiece, nodeTier: NodeTier): CommandBase {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.PrepScore(gamePiece, nodeTier) }
        .andThen(
          WaitUntilCommand {
            isAtRequestedState && currentState == SuperstructureStates.SCORE_PREP
          }
        )
    returnCommand.name = "PrepScore${gamePiece?.name}${nodeTier?.name}Command"
    return returnCommand
  }

  fun groundIntakeConeCommand(): CommandBase {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.GroundIntakeCone() }.until {
        isAtRequestedState && currentState == SuperstructureStates.GROUND_INTAKE_CONE
      }

    returnCommand.name = "GroundIntakeConeCommand"
    return returnCommand
  }

  fun doubleSubConeCommand(): CommandBase {
    val returnCommand =
      runOnce {
        currentRequest = SuperstructureRequest.DoubleSubstationIntakePrep(GamePiece.CONE)
      }
        .until { currentState == SuperstructureStates.DOUBLE_SUBSTATION_CLEANUP }

    returnCommand.name = "DoubleSubIntakeConeCommand"
    return returnCommand
  }

  fun singleSubConeCommand(): CommandBase {
    val returnCommand =
      runOnce {
        currentRequest = SuperstructureRequest.SingleSubstationIntakePrep(GamePiece.CONE)
      }
        .until { currentState == SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CLEANUP }

    returnCommand.name = "SingleSubIntakeConeCommand"
    return returnCommand
  }

  fun score(): CommandBase {
    var stateToBeChecked: SuperstructureStates
    if (usingGamePiece == GamePiece.CUBE) {
      stateToBeChecked = SuperstructureStates.SCORE_CUBE
    } else {
      stateToBeChecked = SuperstructureStates.SCORE_CONE
    }

    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.Score() }
        .andThen(
          WaitUntilCommand {
            if (DriverStation.isAutonomous()) {
              canMoveSafely &&
                isAtRequestedState &&
                currentState == SuperstructureStates.SCORE_CLEANUP
            } else {
              isAtRequestedState && currentState == SuperstructureStates.IDLE
            }
          }
        )

    returnCommand.name = "ScoreCommand"
    return returnCommand
  }

  fun regenerateProfiles() {
    elevator.regenerateProfileNextLoopCycle()
    groundIntake.regenerateProfileNextLoopCycle()
    manipulator.regenerateProfileNextLoopCycle()
  }

  // Elevator commands
  fun homeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest = Request.ElevatorRequest.Home()
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorHomeCommand"
    return returnCommand
  }

  fun elevatorCharacterizeCommand(): CommandBase {
    return ElevatorKsCharacterizeCommand(this)
  }

  fun elevatorSetVoltage(voltage: ElectricalPotential) {
    elevator.currentRequest = Request.ElevatorRequest.OpenLoop(voltage)
    currentRequest = SuperstructureRequest.Tuning()
  }

  fun elevatorOpenLoopExtendCommand(): CommandBase {
    val returnCommand =
      run {
        elevator.currentRequest =
          Request.ElevatorRequest.OpenLoop(
            Elevator.TunableElevatorHeights.openLoopExtendVoltage.get()
          )
        currentRequest = SuperstructureRequest.Tuning()
      }
        .andThen(
          runOnce {
            elevator.currentRequest =
              Request.ElevatorRequest.TargetingPosition(elevatorInputs.elevatorPosition)
          }
        )

    returnCommand.name = "ElevatorOpenLoopExtendCommand"
    return returnCommand
  }

  fun elevatorOpenLoopRetractCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.OpenLoop(
          Elevator.TunableElevatorHeights.openLoopRetractVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorOpenLoopRetractCommand"
    return returnCommand
  }

  fun elevatorGoToLowCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.cubeDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToLowCubeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToLowConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.coneDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToLowConeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToMidCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.midCubeHeight.get() +
            Elevator.TunableElevatorHeights.cubeDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToMidCubeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToMidConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.midConeHeight.get() +
            Elevator.TunableElevatorHeights.coneDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToMidConeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToHighCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.highCubeHeight.get() +
            Elevator.TunableElevatorHeights.cubeDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToHighCubeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToHighConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.highConeHeight.get() +
            Elevator.TunableElevatorHeights.coneDropPosition.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToHighConeNodeCommand"
    return returnCommand
  }

  fun elevatorGoToIntakeCubeFromDoubleStationCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.doubleSubstationHeight.get() +
            Elevator.TunableElevatorHeights.shelfIntakeCubeOffset.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToIntakeCubeFromDoubleStationCommand"
    return returnCommand
  }

  fun elevatorGoToIntakeConeFromDoubleStationCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.doubleSubstationHeight.get() +
            Elevator.TunableElevatorHeights.shelfIntakeConeOffset.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorGoToIntakeConeFromDoubleStationCommand"
    return returnCommand
  }

  fun elevatorSlamConeOnMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.midConeHeight.get(),
          Elevator.TunableElevatorHeights.slamVelocity.get()
        ) // TODO fix height?
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorSlamConeOnMidNodeCommand"
    return returnCommand
  }

  fun elevatorSlamConeOnHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.highConeHeight.get(),
          Elevator.TunableElevatorHeights.slamVelocity.get()
        ) // TODO fix height?
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ElevatorSlamConeOnHighNodeCommand"
    return returnCommand
  }

  // Ground Intake Commands
  fun groundIntakeIntakeCommand(): CommandBase {
    val returnCommand = runOnce {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.intakeAngle.get(),
          GroundIntake.TunableGroundIntakeStates.intakeVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun groundIntakeOuttakeCommand(): CommandBase {
    val returnCommand = runOnce {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.outtakeAngle.get(),
          GroundIntake.TunableGroundIntakeStates.outtakeVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GroundIntakeOuttakeCommand"
    return returnCommand
  }

  fun groundIntakeSetArmVoltage(voltage: ElectricalPotential) {
    groundIntake.currentRequest = Request.GroundIntakeRequest.OpenLoop(voltage, 0.0.volts)
    currentRequest = SuperstructureRequest.Tuning()
  }

  fun groundIntakeCharacterizeCommand(): CommandBase {
    return GroundIntakeCharacterizeCommand(this)
  }

  fun groundIntakeStowedUpCommand(): CommandBase {
    val returnCommand = run {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(), 0.0.volts
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun groundIntakeStowedDownCommand(): CommandBase {
    val returnCommand = run {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
          GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun groundIntakeOpenLoopRetractCommand(): CommandBase {
    val returnCommand =
      run {
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.OpenLoop(10.0.volts, 0.0.volts)
        currentRequest = SuperstructureRequest.Tuning()
      }
        .andThen(runOnce { currentRequest = SuperstructureRequest.Idle() })

    returnCommand.name = "GroundIntakeOpenLoopRetractCommand"
    return returnCommand
  }

  fun groundIntakeIntakeCubeCommand(): CommandBase {
    val returnCommand =
      run {
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.intakeVoltage.get()
          )
        manipulator.currentRequest =
          Request.ManipulatorRequest.OpenLoop(
            0.0.volts, Manipulator.TunableManipulatorStates.cubeInVoltage.get()
          )

        currentRequest = SuperstructureRequest.Tuning()
      }
        .andThen(runOnce { currentRequest = SuperstructureRequest.Idle() })

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun groundIntakeZeroArm() {
    groundIntake.zeroArm()
  }

  fun manipulatorSetArmVoltage(voltage: ElectricalPotential) {
    manipulator.currentRequest = Request.ManipulatorRequest.OpenLoop(voltage, 0.0.volts)
    currentRequest = SuperstructureRequest.Tuning()
  }

  fun manipulatorOpenLoopIntakeConeCommand(): CommandBase {
    val returnCommand = run {
      manipulator.currentRequest =
        Request.ManipulatorRequest.OpenLoop(
          ManipulatorConstants.IDLE_VOLTAGE,
          Manipulator.TunableManipulatorStates.coneInVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ManipulatorOpenLoopIntakeConeCommand"
    return returnCommand
  }

  fun manipulatorOpenLoopExtendCommand(): CommandBase {
    val returnCommand = run {
      manipulator.currentRequest =
        Request.ManipulatorRequest.OpenLoop(
          Manipulator.TunableManipulatorStates.openLoopExtendVoltage.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ManipulatorOpenLoopExtendCommand"
    return returnCommand
  }

  fun manipulatorOpenLoopRetractCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.OpenLoop(
          Manipulator.TunableManipulatorStates.openLoopRetractVoltage.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ManipulatorOpenLoopRetractCommand"
    return returnCommand
  }

  fun manipulatorGoToMinExtensionCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.minExtension.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GoToMinExtensionCommand"
    return returnCommand
  }

  fun manipulatorGoToMaxExtensionCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.maxExtension.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "GoToMaxExtensionCommand"
    return returnCommand
  }

  fun manipulatorIntakeCubeFromDoubleSubstationCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.doubleSubstationIntakeShelfExtension.get(),
          Manipulator.TunableManipulatorStates.cubeInVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "IntakeCubeFromDoubleSubstationCommand"
    return returnCommand
  }

  fun manipulatorIntakeConeFromDoubleSubstationCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.doubleSubstationIntakeShelfExtension.get(),
          Manipulator.TunableManipulatorStates.coneInVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "IntakeConeFromDoubleSubstationCommand"
    return returnCommand
  }

  fun manipulatorScoreCubeAtHybridNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.lowConeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.cubeOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreCubeAtHybridNodeCommand"
    return returnCommand
  }

  fun manipulatorScoreConeAtHybridNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.lowConeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.coneOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreConeAtHybridNodeCommand"
    return returnCommand
  }

  fun manipulatorScoreConeAtMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.midCubeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.coneOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreConeAtMidNodeCommand"
    return returnCommand
  }

  fun manipulatorScoreCubeAtMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.midCubeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.cubeOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreCubeAtMidNodeCommand"
    return returnCommand
  }

  fun manipulatorScoreCubeAtHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.highCubeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.cubeOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreConeAtHighNodeCommand"
    return returnCommand
  }

  fun manipulatorScoreConeAtHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      manipulator.currentRequest =
        Request.ManipulatorRequest.TargetingPosition(
          Manipulator.TunableManipulatorStates.highCubeScoreExtension.get(),
          Manipulator.TunableManipulatorStates.coneOutVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }

    returnCommand.name = "ScoreConeAtHighNodeCommand"
    return returnCommand
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      IDLE, // maybe separate idle state for ground intake stowed out vs in
      HOME_PREP,
      HOME,
      GROUND_INTAKE_CUBE_PREP,
      GROUND_INTAKE_CUBE,
      GROUND_INTAKE_CUBE_CLEANUP,
      GROUND_INTAKE_CONE_PREP,
      GROUND_INTAKE_CONE,
      EJECT_GAME_PIECE,
      GROUND_INTAKE_CONE_CLEANUP,
      DOUBLE_SUBSTATION_INTAKE_PREP,
      DOUBLE_SUBSTATION_INTAKE,
      DOUBLE_SUBSTATION_CLEANUP,
      SINGLE_SUBSTATION_INTAKE_PREP,
      SINGLE_SUBSTATION_INTAKE_CUBE,
      SINGLE_SUBSTATION_INTAKE_CONE,
      SINGLE_SUBSTATION_INTAKE_CLEANUP,
      SCORE_PREP,
      SCORE_CUBE,
      SCORE_CONE,
      SCORE_CLEANUP
    }
  }
}
