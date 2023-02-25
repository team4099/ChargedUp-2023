package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.Robot
import com.team4099.robot2023.commands.elevator.ElevatorKsCharacterizeCommand
import com.team4099.robot2023.commands.elevator.GroundIntakeCharacterizeCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.groundintake.GroundIntake
import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
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
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.volts
import com.team4099.robot2023.subsystems.superstructure.Request.SuperstructureRequest as SuperstructureRequest

class Superstructure(
  private val elevator: Elevator,
  private val groundIntake: GroundIntake,
  private val manipulator: Manipulator
) : SubsystemBase() {

  var currentRequest: SuperstructureRequest = SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is SuperstructureRequest.GroundIntakeCone -> usingGamePiece = GamePiece.CONE
        is SuperstructureRequest.GroundIntakeCube -> usingGamePiece = GamePiece.CUBE
        is SuperstructureRequest.DoubleSubstationIntake -> usingGamePiece = value.gamePiece
        is SuperstructureRequest.SingleSubstationIntake -> usingGamePiece = value.gamePiece
        is SuperstructureRequest.PrepScore -> {
          usingGamePiece = value.gamePiece
          nodeTier = value.nodeTier
        }
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

  var nodeTier: NodeTier = NodeTier.NONE

  var lastTransitionTime = Clock.fpgaTime

  val elevatorInputs = elevator.inputs

  val groundIntakeInputs = groundIntake.inputs

  val manipulatorInputs = manipulator.inputs

  override fun periodic() {
    val elevatorLoopStartTime = Clock.fpgaTime
    elevator.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/ElevatorLoopTimeMS",
        (Clock.fpgaTime - elevatorLoopStartTime).inMilliseconds
      )

    val groundIntakeLoopStartTime = Clock.fpgaTime
    groundIntake.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/GroundIntakeLoopTimeMS",
        (Clock.fpgaTime - groundIntakeLoopStartTime).inMilliseconds
      )

    val manipulatorLoopStartTime = Clock.fpgaTime
    manipulator.periodic()
    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/ManipulatorLoopTimeMS",
        (Clock.fpgaTime - manipulatorLoopStartTime).inMilliseconds
      )

    Logger.getInstance()
      .recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    Logger.getInstance().recordOutput("Superstructure/currentState", currentState.name)
    Logger.getInstance().recordOutput("Superstructure/usingGamePiece", usingGamePiece.name)
    Logger.getInstance()
      .recordOutput("Superstructure/holdingGamePiece", manipulator.holdingGamePiece.name)
    Logger.getInstance().recordOutput("Superstructure/nodeTier", nodeTier.name)
    Logger.getInstance()
      .recordOutput("Superstructure/lastTransitionTime", lastTransitionTime.inSeconds)

    Logger.getInstance()
      .recordOutput(
        "SimulatedMechanisms/0",
        Pose3d(
          0.1016.meters,
          0.0.meters,
          0.211550.meters,
          Rotation3d(0.0.degrees, -groundIntake.inputs.armPosition, 0.0.degrees)
        )
          .pose3d
      )

    if (elevator.inputs.elevatorPosition >= ElevatorConstants.FIRST_STAGE_HEIGHT) {
      Logger.getInstance()
        .recordOutput(
          "SimulatedMechanisms/1",
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
        )
    } else {
      Logger.getInstance()
        .recordOutput(
          "SimulatedMechanisms/1",
          Pose3d()
            .transformBy(
              Transform3d(
                Translation3d(0.0.inches, 0.0.inches, 0.0.inches)
                  .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
                Rotation3d()
              )
            )
            .pose3d
        )
    }

    Logger.getInstance()
      .recordOutput(
        "SimulatedMechanisms/2",
        Pose3d()
          .transformBy(
            Transform3d(
              Translation3d(0.0.inches, 0.0.inches, elevator.inputs.elevatorPosition)
                .rotateBy(Rotation3d(0.0.degrees, 40.5.degrees, 0.0.degrees)),
              Rotation3d()
            )
          )
          .pose3d
      )

    Logger.getInstance()
      .recordOutput(
        "SimulatedMechanisms/3",
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
        // Outputs
        val rollerVoltage =
          when (manipulator.holdingGamePiece) {
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
        if (groundIntake.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.TargetingPosition(
              Elevator.TunableElevatorHeights.minPosition.get()
            )
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.minExtension.get(), rollerVoltage
            )
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
              SuperstructureStates.DOUBLE_SUBSTATION_INTAKE_PREP
            is SuperstructureRequest.GroundIntakeCone ->
              SuperstructureStates.GROUND_INTAKE_CONE_PREP
            is SuperstructureRequest.GroundIntakeCube ->
              SuperstructureStates.GROUND_INTAKE_CUBE_PREP
            is SuperstructureRequest.Home -> SuperstructureStates.HOME_PREP
            is SuperstructureRequest.Idle -> SuperstructureStates.IDLE
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
            is SuperstructureRequest.SingleSubstationIntake -> {
              when (usingGamePiece) {
                GamePiece.CONE -> SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CONE
                GamePiece.CUBE -> SuperstructureStates.SINGLE_SUBSTATION_INTAKE_CUBE
                GamePiece.NONE -> SuperstructureStates.IDLE
              }
            }
            is SuperstructureRequest.Tuning -> SuperstructureStates.TUNING
            else -> currentState
          }
      }
      SuperstructureStates.HOME_PREP -> {
        // Outputs
        groundIntake.currentRequest = Request.GroundIntakeRequest.ZeroArm()


        if (groundIntake.isZeroed){
          groundIntake.currentRequest =
            Request.GroundIntakeRequest.TargetingPosition(
              //GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
              4.4.degrees,
              GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
            )
        }

        // Transition
        if (groundIntake.isAtTargetedPosition && groundIntake.isZeroed) {
          nextState = SuperstructureStates.HOME
        }
      }
      SuperstructureStates.HOME -> {
        // Outputs
        elevator.currentRequest = Request.ElevatorRequest.Home()
        manipulator.currentRequest = Request.ManipulatorRequest.Home()

        // Transition
        if (elevator.isHomed && manipulator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE_PREP -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.intakeAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.groundIntakeCubeExtension.get(),
              ManipulatorConstants.IDLE_VOLTAGE
            )
        }

        // Transition
        if (groundIntake.isAtTargetedPosition && manipulator.isAtTargetedPosition) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCube) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.intakeAngle.get(),
            GroundIntake.TunableGroundIntakeStates.intakeVoltage.get()
          )
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.groundIntakeCubeExtension.get(),
            Manipulator.TunableManipulatorStates.cubeInVoltage.get()
          )

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          currentRequest !is SuperstructureRequest.GroundIntakeCube
        ) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCube) {
          nextState = SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CUBE_CLEANUP -> {
        // Goes immediately to IDLE
        // Outputs

        // Transition
        nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.GROUND_INTAKE_CONE_PREP -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.groundIntakeConeExtension.get(),
              ManipulatorConstants.IDLE_VOLTAGE
            )
        }

        // Transition
        if (groundIntake.isAtTargetedPosition && manipulator.isAtTargetedPosition) {
          nextState = SuperstructureStates.GROUND_INTAKE_CONE
        } else if (currentRequest !is SuperstructureRequest.GroundIntakeCone) {
          nextState = SuperstructureStates.GROUND_INTAKE_CONE_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CONE -> {
        // Outputs
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.groundIntakeConeExtension.get(),
            Manipulator.TunableManipulatorStates.coneInVoltage.get()
          )

        // Transition
        if (manipulator.isAtTargetedPosition &&
          currentRequest !is SuperstructureRequest.GroundIntakeCone
        ) {
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

        // Transition
        if (manipulator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.intakeTime.get()
        ) {
          currentRequest = SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.DOUBLE_SUBSTATION_INTAKE_PREP -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.doubleSubstationIntakeShelfExtension.get(),
              ManipulatorConstants.IDLE_VOLTAGE
            )

          if (manipulator.isAtTargetedPosition){
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
          }
        }

        // Transition
        if (groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.DOUBLE_SUBSTATION_INTAKE
        } else if (currentRequest !is SuperstructureRequest.DoubleSubstationIntake) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.DOUBLE_SUBSTATION_INTAKE -> {
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
        if (manipulator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.intakeTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
          currentRequest = SuperstructureRequest.Idle()
        }
      }
      SuperstructureStates.SINGLE_SUBSTATION_INTAKE_PREP -> {
        // Outputs
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition) {
          manipulator.currentRequest =
            Request.ManipulatorRequest.TargetingPosition(
              Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
              ManipulatorConstants.IDLE_VOLTAGE
            )
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
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.singleSubstationIntakeShelfExtension.get(),
            Manipulator.TunableManipulatorStates.coneInVoltage.get()
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
      SuperstructureStates.SCORE_PREP -> {
        groundIntake.currentRequest =
          Request.GroundIntakeRequest.TargetingPosition(
            GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
            GroundIntake.TunableGroundIntakeStates.neutralVoltage.get()
          )
        if (groundIntake.isAtTargetedPosition) {
          val scoreHeight =
            when (nodeTier) {
              NodeTier.HYBRID -> {
                when (usingGamePiece) {
                  GamePiece.CUBE ->
                    Elevator.TunableElevatorHeights.hybridHeight.get() +
                      Elevator.TunableElevatorHeights.cubeDropPosition.get()
                  GamePiece.CONE ->
                    Elevator.TunableElevatorHeights.hybridHeight.get() +
                      Elevator.TunableElevatorHeights.coneDropPosition.get()
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

          if (elevator.isAtTargetedPosition) {
            val extension =
              when (nodeTier) {
                NodeTier.HYBRID -> Manipulator.TunableManipulatorStates.lowScoreExtension.get()
                NodeTier.MID -> Manipulator.TunableManipulatorStates.midScoreExtension.get()
                NodeTier.HIGH -> Manipulator.TunableManipulatorStates.highScoreExtension.get()
                else -> 0.0.inches
              }

            when (usingGamePiece){
              Constants.Universal.GamePiece.NONE -> manipulator.currentRequest =
                Request.ManipulatorRequest.TargetingPosition(
                  extension, ManipulatorConstants.IDLE_VOLTAGE
                )
              Constants.Universal.GamePiece.CONE -> manipulator.currentRequest =
                Request.ManipulatorRequest.TargetingPosition(
                  extension, ManipulatorConstants.CONE_IDLE
                )
              Constants.Universal.GamePiece.CUBE -> manipulator.currentRequest =
                Request.ManipulatorRequest.TargetingPosition(
                  extension, ManipulatorConstants.CUBE_IDLE
                )
            }

          }
        }

        // Transition
        if (currentRequest is SuperstructureRequest.Score &&
          groundIntake.isAtTargetedPosition &&
          manipulator.isAtTargetedPosition &&
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
          nextState = SuperstructureStates.SCORE_CLEANUP
          currentRequest = SuperstructureRequest.Idle()
        } else if (currentRequest !is SuperstructureRequest.Score) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_CONE -> {
        // Outputs
        val dropToPosition =
          when (nodeTier) {
            NodeTier.HYBRID -> Elevator.TunableElevatorHeights.hybridHeight.get()  + Elevator.TunableElevatorHeights.coneDropPosition.get()
            NodeTier.MID -> Elevator.TunableElevatorHeights.midConeHeight.get()  + Elevator.TunableElevatorHeights.coneDropPosition.get()
            NodeTier.HIGH -> Elevator.TunableElevatorHeights.highConeHeight.get()  + Elevator.TunableElevatorHeights.coneDropPosition.get()
            else -> elevator.elevatorPositionTarget
          }
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
          manipulator.isAtTargetedPosition &&
          (Clock.fpgaTime - lastTransitionTime) >=
          Manipulator.TunableManipulatorStates.spitTime.get()
        ) {
          nextState = SuperstructureStates.SCORE_CLEANUP
          currentRequest = SuperstructureRequest.Idle()
        } else if (currentRequest !is SuperstructureRequest.Score) {
          nextState = SuperstructureStates.SCORE_CLEANUP
        }
      }
      SuperstructureStates.SCORE_CLEANUP -> {
        manipulator.currentRequest =
          Request.ManipulatorRequest.TargetingPosition(
            Manipulator.TunableManipulatorStates.minExtension.get(),
            ManipulatorConstants.IDLE_VOLTAGE
          )

        if (manipulator.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.TUNING -> {

        if (currentRequest is SuperstructureRequest.Idle){
          nextState = SuperstructureStates.IDLE
        }
      }
    }

    if (nextState != currentState) {
      lastTransitionTime = Clock.fpgaTime
    }

    currentState = nextState
  }

  // Superstructure Commands
  fun intakeConeFromDoubleSubStationCommand(): CommandBase{
    val returnCommand =
      runOnce{
        currentRequest = SuperstructureRequest.DoubleSubstationIntake(
          Constants.Universal.GamePiece.CONE
        )
      }.until { currentState == SuperstructureStates.DOUBLE_SUBSTATION_INTAKE }

    returnCommand.name = "IntakeConeFromDoubleSubStationCommand"
    return returnCommand
  }

  fun prepscoreConeHighCommand(): CommandBase {
    val returnCommand =
      runOnce {
        currentRequest =
          SuperstructureRequest.PrepScore(
            Constants.Universal.GamePiece.CONE, Constants.Universal.NodeTier.MID
          )
      }
        .until { currentState == SuperstructureStates.SCORE_PREP }

    returnCommand.name = "PrepscoreConeHighCommand"
    return returnCommand
  }

  fun scoreConeHighCommand(): CommandBase {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.Score() }.until {
        currentState == SuperstructureStates.SCORE_CONE
      }

    returnCommand.name = "ScoreConeHighCommand"
    return returnCommand
  }

  fun regenerateProfiles(){
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

  fun elevatorCharacterizeCommand(): CommandBase{
    return ElevatorKsCharacterizeCommand(this)
  }

  fun elevatorSetVoltage(voltage: ElectricalPotential){
    elevator.currentRequest = Request.ElevatorRequest.OpenLoop(voltage)
    currentRequest = SuperstructureRequest.Tuning()
  }

  fun elevatorOpenLoopExtendCommand(): CommandBase {
    val returnCommand = run {
      elevator.currentRequest =
        Request.ElevatorRequest.OpenLoop(
          Elevator.TunableElevatorHeights.openLoopExtendVoltage.get()
        )
      currentRequest = SuperstructureRequest.Tuning()
    }.andThen(
      runOnce { elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(elevatorInputs.elevatorPosition) }
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

  fun groundIntakeSetArmVoltage(voltage: ElectricalPotential){
    groundIntake.currentRequest = Request.GroundIntakeRequest.OpenLoop(voltage, 0.0.volts)
    currentRequest = SuperstructureRequest.Tuning()
  }

  fun groundIntakeCharacterizeCommand(): CommandBase{
    return GroundIntakeCharacterizeCommand(this)
  }

  fun groundIntakeStowedUpCommand(): CommandBase {
    val returnCommand = run {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.stowedUpAngle.get(),
          0.0.volts
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
    val returnCommand = run {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.OpenLoop(
          0.676.volts,
          0.0.volts
        )
      currentRequest = SuperstructureRequest.Tuning()
    }.andThen (
      runOnce{
        currentRequest = SuperstructureRequest.Idle()})


    returnCommand.name = "GroundIntakeOpenLoopRetractCommand"
    return returnCommand
  }

  fun groundIntakeIntakeCubeCommand(): CommandBase {
    val returnCommand = run {
      groundIntake.currentRequest =
        Request.GroundIntakeRequest.TargetingPosition(
          GroundIntake.TunableGroundIntakeStates.stowedDownAngle.get(),
          GroundIntake.TunableGroundIntakeStates.intakeVoltage.get()
        )
      manipulator.currentRequest =
        Request.ManipulatorRequest.OpenLoop(
          0.0.volts,
          Manipulator.TunableManipulatorStates.cubeInVoltage.get()

        )
      currentRequest = SuperstructureRequest.Tuning()
    }.andThen (
      runOnce{
      currentRequest = SuperstructureRequest.Idle()})


    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun groundIntakeZeroArm(){
    groundIntake.zeroArm()
  }

  fun manipulatorSetArmVoltage(voltage: ElectricalPotential){
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
          Manipulator.TunableManipulatorStates.lowScoreExtension.get(),
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
          Manipulator.TunableManipulatorStates.lowScoreExtension.get(),
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
          Manipulator.TunableManipulatorStates.midScoreExtension.get(),
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
          Manipulator.TunableManipulatorStates.midScoreExtension.get(),
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
          Manipulator.TunableManipulatorStates.highScoreExtension.get(),
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
          Manipulator.TunableManipulatorStates.highScoreExtension.get(),
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
      GROUND_INTAKE_CONE_CLEANUP,
      DOUBLE_SUBSTATION_INTAKE_PREP,
      DOUBLE_SUBSTATION_INTAKE,
      SINGLE_SUBSTATION_INTAKE_PREP,
      SINGLE_SUBSTATION_INTAKE_CUBE,
      SINGLE_SUBSTATION_INTAKE_CONE,
      SCORE_PREP,
      SCORE_CUBE,
      SCORE_CONE,
      SCORE_CLEANUP
    }
  }
}
