package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.MechanismSimConstants.carriageAttachment
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteXPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteYPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.secondStageAttachment
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.RequestStructure.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()

  // PID and Feedforward Values
  var elevatorFeedforward: ElevatorFeedforward

  private val kP =
    LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  object TunableElevatorHeights {
    val minHeight =
      LoggedTunableValue(
        "Elevator/minHeight",
        ElevatorConstants.ELEVATOR_MAX_RETRACTION,
        Pair({ it.inInches }, { it.inches })
      )

    val maxHeight =
      LoggedTunableValue(
        "Elevator/maxHeight",
        ElevatorConstants.ELEVATOR_MAX_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage", 12.0.volts, Pair({ it.inVolts }, { it.volts })
      )

    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage", -12.0.volts, Pair({ it.inVolts }, { it.volts })
      )

    val cubeDropHeight =
      LoggedTunableValue(
        "Elevator/cubeDropHeight",
        ElevatorConstants.CUBE_DROP_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val coneDropHeight =
      LoggedTunableValue(
        "Elevator/coneDropHeight",
        ElevatorConstants.CONE_DROP_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val slamVelocity =
      LoggedTunableValue(
        "Elevator/slamVelocity",
        ElevatorConstants.SLAM_VELOCITY,
        Pair({ it.inInchesPerSecond }, { it.inches.perSecond })
      )

    val shelfIntakeCubeOffset =
      LoggedTunableValue(
        "Elevator/shelfIntakeCubeOffset",
        ElevatorConstants.INTAKE_CUBE_SHELF_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val shelfIntakeConeOffset =
      LoggedTunableValue(
        "Elevator/shelfIntakeConeOffset",
        ElevatorConstants.INTAKE_CONE_SHELF_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val hybridHeight =
      LoggedTunableValue(
        "Elevator/midCubeHeight", 0.0.inches, Pair({ it.inInches }, { it.inches })
      )

    val midCubeHeight =
      LoggedTunableValue(
        "Elevator/midCubeHeight",
        FieldConstants.Grids.midCubeZ,
        Pair({ it.inInches }, { it.inches })
      )

    val midConeHeight =
      LoggedTunableValue(
        "Elevator/midConeHeight",
        FieldConstants.Grids.midConeZ,
        Pair({ it.inInches }, { it.inches })
      )

    val highCubeHeight =
      LoggedTunableValue(
        "Elevator/highCubeHeight",
        FieldConstants.Grids.highCubeZ,
        Pair({ it.inInches }, { it.inches })
      )

    val highConeHeight =
      LoggedTunableValue(
        "Elevator/highConeHeight",
        FieldConstants.Grids.highConeZ,
        Pair({ it.inInches }, { it.inches })
      )

    val doubleSubstationHeight =
      LoggedTunableValue(
        "Elevator/doubleSubstationHeight",
        FieldConstants.LoadingZone.doubleSubstationShelfZ,
        Pair({ it.inInches }, { it.inches })
      )
  }

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFTLIMIT_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFTLIMIT_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION

  var isHomed = false

  var currentState: ElevatorState = ElevatorState.UNINITIALIZED

  var currentRequest: ElevatorRequest =
    ElevatorRequest.TargetingPosition(TunableElevatorHeights.minHeight.get())

  var elevatorPositionTarget = 0.0.inches

  var elevatorVelocityTarget = 0.0.inches.perSecond

  var elevatorVoltageTarget = 0.0.volts

  private var lastRequestedPosition = 0.0.inches

  private var lastRequestedVelocity = 0.0.inches.perSecond

  private var lastRequestedVoltage = 0.0.volts

  private var timeProfileGeneratedAt = Clock.fpgaTime

  // trapezoidal profile constraints
  private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  private var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)

  private var elevatorProfile =
    TrapezoidProfile(
      elevatorConstraints,
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond),
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond)
    )

  init {
    TunableElevatorHeights

    // initializing pid constants and changing FF for sim vs real
    if (RobotBase.isReal()) {
      isHomed = false

      kP.initDefault(ElevatorConstants.REAL_KP)
      kI.initDefault(ElevatorConstants.REAL_KI)
      kD.initDefault(ElevatorConstants.REAL_KD)

      elevatorFeedforward =
        ElevatorFeedforward(
          ElevatorConstants.REAL_ELEVATOR_KS,
          ElevatorConstants.ELEVATOR_KG,
          ElevatorConstants.ELEVATOR_KV,
          ElevatorConstants.ELEVATOR_KA
        )
    } else {
      isHomed = true

      kP.initDefault(ElevatorConstants.SIM_KP)
      kI.initDefault(ElevatorConstants.SIM_KI)
      kD.initDefault(ElevatorConstants.SIM_KD)

      elevatorFeedforward =
        ElevatorFeedforward(
          ElevatorConstants.SIM_ELEVATOR_KS,
          ElevatorConstants.ELEVATOR_KG,
          ElevatorConstants.ELEVATOR_KV,
          ElevatorConstants.ELEVATOR_KA
        )
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    updateMech2d()

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Elevator", inputs)

    Logger.getInstance()
      .recordOutput(
        "Elevator/elevatorHeightWithRespectToGround",
        ElevatorConstants.ElevatorStates.fromPositionToHeight(inputs.elevatorPosition).inInches
      )

    Logger.getInstance().recordOutput("Elevator/currentState", currentState.name)
    Logger.getInstance()
      .recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)

    Logger.getInstance()
      .recordOutput("Elevator/elevatorPositionTarget", elevatorPositionTarget.inInches)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorVelocityTarget", elevatorVelocityTarget.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorVoltageTarget", elevatorVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("Elevator/lastElevatorPositionTarget", lastRequestedPosition.inInches)
    Logger.getInstance()
      .recordOutput(
        "Elevator/lastElevatorVelocityTarget", lastRequestedVelocity.inInchesPerSecond
      )
    Logger.getInstance()
      .recordOutput("Elevator/lastElevatorVoltageTarget", lastRequestedVoltage.inVolts)

    Logger.getInstance().recordOutput("Elevator/forwardLimitReached", forwardLimitReached)
    Logger.getInstance().recordOutput("Elevator/reverseLimitReached", reverseLimitReached)

    var nextState = currentState
    when (currentState) {
      ElevatorState.UNINITIALIZED -> {
        // Outputs

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.OPEN_LOOP -> {
        // Outputs
        setOutputVoltage(elevatorVoltageTarget)

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.TARGETING_POSITION -> {
        // Outputs
        if (elevatorPositionTarget != lastRequestedPosition ||
          elevatorVelocityTarget != lastRequestedVelocity
        ) {
          elevatorProfile =
            TrapezoidProfile(
              elevatorConstraints,
              TrapezoidProfile.State(elevatorPositionTarget, elevatorVelocityTarget),
              TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
            )
          timeProfileGeneratedAt = Clock.fpgaTime
          lastRequestedPosition = elevatorPositionTarget
          lastRequestedVelocity = elevatorVelocityTarget
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        setPosition(elevatorProfile.calculate(timeElapsed))

        Logger.getInstance()
          .recordOutput(
            "Elevator/completedMotionProfile", elevatorProfile.isFinished(timeElapsed)
          )

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)

        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastRequestedPosition = -1337.inches
          lastRequestedVelocity = -1337.inches.perSecond
        }
      }
      ElevatorState.HOME -> {
        // Outputs
        if (!isHomed &&
          inputs.elevatorPosition < ElevatorConstants.HOMING_POSITION_THRESHOLD &&
          inputs.leaderStatorCurrent > ElevatorConstants.HOMING_STALL_CURRENT
        ) {
          setOutputVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
        } else {
          zeroEncoder()
          isHomed = true
        }

        // Transition
        nextState = fromElevatorRequestToState(currentRequest)
      }
    }

    currentState = nextState
    when (val typedRequest = currentRequest) {
      is ElevatorRequest.OpenLoop -> elevatorVoltageTarget = typedRequest.voltage
      is ElevatorRequest.TargetingPosition -> {
        elevatorPositionTarget = typedRequest.position
        elevatorVelocityTarget = typedRequest.finalVelocity
      }
    }
  }

  /**
   * Sets the voltage of the elevator motors but also checks to make sure elevator doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setOutputVoltage(voltage: ElectricalPotential) {
    if (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts) {
      io.setOutputVoltage(0.volts)
    } else {
      io.setOutputVoltage(voltage)
    }
  }

  /**
   * Sets the elevator to a specific position using trapezoidal profile state and feedforward also
   * has safety for max extension and retractions
   *
   * @param voltage the voltage to set the motor to
   */
  fun setPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAccel =
      ((setpoint.velocity - elevatorSetpoint.velocity) / (Constants.Universal.LOOP_PERIOD_TIME))

    elevatorSetpoint = setpoint

    var feedforward = elevatorFeedforward.calculate(setpoint.velocity, elevatorAccel)

    if (forwardLimitReached && setpoint.position > inputs.elevatorPosition ||
      reverseLimitReached && setpoint.position < inputs.elevatorPosition
    ) {
      io.setOutputVoltage(0.volts)
    } else {
      io.setPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("Elevator/targetPosition", setpoint.position.inInches)
    Logger.getInstance().recordOutput("Elevator/targetVel", setpoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorAcceleration", elevatorAccel.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Elevator/elevatorFeedForward", feedforward.inVolts)
  }

  /** set the current encoder position to be the encoders zero value */
  fun zeroEncoder() {
    io.zeroEncoder()
  }

  fun defaultCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest = ElevatorRequest.TargetingPosition(inputs.elevatorPosition)
    }

    returnCommand.name = "ElevatorDefaultCommand"
    return returnCommand
  }

  fun homeCommand(): CommandBase {
    val returnCommand = runOnce { currentRequest = ElevatorRequest.Home() }

    returnCommand.name = "ElevatorHomeCommand"
    return returnCommand
  }

  fun openLoopExtendCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest = ElevatorRequest.OpenLoop(TunableElevatorHeights.openLoopExtendVoltage.get())
    }

    returnCommand.name = "ElevatorOpenLoopExtendCommand"
    return returnCommand
  }

  fun openLoopRetractCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest = ElevatorRequest.OpenLoop(TunableElevatorHeights.openLoopRetractVoltage.get())
    }

    returnCommand.name = "ElevatorOpenLoopRetractCommand"
    return returnCommand
  }

  fun goToLowCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(TunableElevatorHeights.cubeDropHeight.get())
    }

    returnCommand.name = "ElevatorGoToLowCubeNodeCommand"
    return returnCommand
  }

  fun goToLowConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(TunableElevatorHeights.coneDropHeight.get())
    }

    returnCommand.name = "ElevatorGoToLowConeNodeCommand"
    return returnCommand
  }

  fun goToMidCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.midCubeHeight.get() +
            TunableElevatorHeights.cubeDropHeight.get()
        )
    }

    returnCommand.name = "ElevatorGoToMidCubeNodeCommand"
    return returnCommand
  }

  fun goToMidConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.midConeHeight.get() +
            TunableElevatorHeights.coneDropHeight.get()
        )
    }

    returnCommand.name = "ElevatorGoToMidConeNodeCommand"
    return returnCommand
  }

  fun goToHighCubeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.highCubeHeight.get() +
            TunableElevatorHeights.cubeDropHeight.get()
        )
    }

    returnCommand.name = "ElevatorGoToHighCubeNodeCommand"
    return returnCommand
  }

  fun goToHighConeNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.highConeHeight.get() +
            TunableElevatorHeights.coneDropHeight.get()
        )
    }

    returnCommand.name = "ElevatorGoToHighConeNodeCommand"
    return returnCommand
  }

  fun goToIntakeCubeFromDoubleStationCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.doubleSubstationHeight.get() +
            TunableElevatorHeights.shelfIntakeCubeOffset.get()
        )
    }

    returnCommand.name = "ElevatorGoToIntakeCubeFromDoubleStationCommand"
    return returnCommand
  }

  fun goToIntakeConeFromDoubleStationCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.doubleSubstationHeight.get() +
            TunableElevatorHeights.shelfIntakeConeOffset.get()
        )
    }

    returnCommand.name = "ElevatorGoToIntakeConeFromDoubleStationCommand"
    return returnCommand
  }

  fun slamConeOnMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.midConeHeight.get(),
          TunableElevatorHeights.slamVelocity.get()
        ) // TODO fix height?
    }

    returnCommand.name = "ElevatorSlamConeOnMidNodeCommand"
    return returnCommand
  }

  fun slamConeOnHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ElevatorRequest.TargetingPosition(
          TunableElevatorHeights.highConeHeight.get(),
          TunableElevatorHeights.slamVelocity.get()
        ) // TODO fix height?
    }

    returnCommand.name = "ElevatorSlamConeOnHighNodeCommand"
    return returnCommand
  }

  //  /**
  //   * Command factory for the elevator idle/hold
  //   *
  //   * @return Command which keeps elevator at its current position uses small amount of
  // feedforward
  //   * to overcome force of gravity
  //   */
  //  fun holdElevatorPosition(): CommandBase {
  //    var positionToHold = inputs.elevatorPosition
  //    val holdPositionCommand =
  //      runOnce {
  //        positionToHold = inputs.elevatorPosition
  //        Logger.getInstance().recordOutput("Elevator/holdPosition", positionToHold.inInches)
  //      }
  //        .andThen(
  //          run {
  //            // check if hold position is close to zero
  //            if (inputs.elevatorPosition.absoluteValue <
  //              ElevatorConstants.ELEVATOR_TOLERANCE
  //            ) {
  //              io.setOutputVoltage(0.0.volts)
  //            } else {
  //              io.setPosition(
  //                positionToHold, elevatorFeedforward.calculate(0.meters.perSecond)
  //              )
  //            }
  //          }
  //        )
  //
  //    holdPositionCommand.name = "ElevatorHoldPositionCommand"
  //    return holdPositionCommand
  //  }
  //
  //  /**
  //   * Inline command to set the elevator to a desired position.
  //   *
  //   * @param position The desired position of the elevator in its frame of reference.
  //   *
  //   * @return A command that runs the elevator's setPosition function until the profile is
  // finished
  //   * running.
  //   */
  //  fun raiseElevatorHeight(state: ElevatorConstants.ElevatorStates): CommandBase {
  //
  //    // Constructing our elevator profile in here because its internal values are dependent on
  // the
  //    // position we want to set the elevator to
  //    val position = Supplier {
  //      ElevatorConstants.ElevatorStates.fromHeightToPosition(
  //        actualElevatorStates[state]?.get() ?: ElevatorConstants.ElevatorStates.MIN_HEIGHT.height
  //      )
  //    }
  //
  //    // Creates a command that runs continuously until the profile is finished. The run function
  //    // accepts a lambda which indicates what we want to run every iteration.
  //    val raiseElevatorHeight = generateElevatorMoveCommand(position, 0.0.inches.perSecond)
  //
  //    raiseElevatorHeight.name = "ElevatorRaiseHeightCommand"
  //    return raiseElevatorHeight
  //  }
  //
  //  fun slamDown(dropDistance: Length): CommandBase {
  //    val slamDownCommand =
  //      generateElevatorMoveCommand(
  //        { inputs.elevatorPosition - dropDistance }, 0.0.inches.perSecond
  //      )
  //    slamDownCommand.name = "ElevatorSlamDownCommand"
  //    return slamDownCommand
  //  }
  //
  //  fun generateElevatorMoveCommand(
  //    targetPosition: Supplier<Length>,
  //    targetVelocity: LinearVelocity
  //  ): CommandBase {
  //    lateinit var elevatorProfile: TrapezoidProfile<Meter>
  //
  //    // Obtaining a start time for this command so that we can pass it into our profile. This is
  // done
  //    // here because we need the startTime to represent the time at which the profile began.
  //    var startTime = Clock.fpgaTime
  //
  //    val elevatorMovementCommand =
  //      runOnce {
  //        startTime = Clock.fpgaTime
  //        desiredHeight = targetPosition.get()
  //        elevatorProfile =
  //          TrapezoidProfile(
  //            elevatorConstraints,
  //            TrapezoidProfile.State(targetPosition.get(), targetVelocity),
  //            TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
  //          )
  //        Logger.getInstance().recordOutput("Elevator/isAtSetpoint", false)
  //      }
  //        .andThen(
  //          run {
  //            setPosition(
  //              elevatorProfile.calculate(
  //                Clock.fpgaTime -
  //                  startTime
  //              )
  //            ) // Every loop cycle we have a different profile state
  //            // we're
  //            // calculating. Hence, we want to pass in a different Trapezoidal
  //            // Profile State into the setPosition function.
  //            Logger.getInstance()
  //              .recordOutput(
  //                "/Elevator/isAtSetpoint",
  //                (inputs.elevatorPosition - targetPosition.get()).absoluteValue <
  //                  ElevatorConstants.ELEVATOR_TOLERANCE
  //              )
  //          }
  //            .until { // The following lambda creates a race condition that stops the command
  //              // we
  //              // created
  //              // above when the passed in condition is true. In our case it's checking when
  //              // elevatorProfile is finished based on elapsed time.
  //              elevatorProfile.isFinished((Clock.fpgaTime - startTime)) ||
  //                (
  //                  forwardLimitReached &&
  //                    targetPosition.get() >
  //                    ElevatorConstants.ELEVATOR_SOFTLIMIT_EXTENSION ||
  //                    reverseLimitReached &&
  //                    targetPosition.get() <
  //                    ElevatorConstants.ELEVATOR_SOFTLIMIT_RETRACTION
  //                  )
  //              // This is the race condition we're passing in.
  //            }
  //        )
  //    return elevatorMovementCommand
  //  }
  //
  //  /**
  //   * Command factory for setting elevator motors to desired voltage
  //   *
  //   * @return Command which sets output voltage of both motors and ends if elevator reachs its
  // limit
  //   */
  //  fun openLoopControl(voltage: ElectricalPotential): Command {
  //    val openLoopElevatorCommand =
  //      run {
  //        setOutputVoltage(voltage)
  //        if (voltage > 0.volts) {
  //          Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopExtend", true)
  //        } else {
  //          Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopRetract", true)
  //        }
  //      }
  //        .until {
  //          (
  //            forwardOpenLoopLimitReached && voltage > 0.volts ||
  //              reverseOpenLoopLimitReached && voltage < 0.volts
  //            )
  //        }
  //        .finallyDo {
  //          setOutputVoltage(0.volts)
  //          if (voltage > 0.volts) {
  //            Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopExtend", false)
  //          } else {
  //            Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopRetract", false)
  //          }
  //        }
  //
  //    if (voltage > 0.volts) {
  //      openLoopElevatorCommand.name = "ElevatorExtendOpenLoopCommand"
  //    } else if (voltage < 0.volts) {
  //      openLoopElevatorCommand.name = "ElevatorRetractOpenLoopCommand"
  //    } else {
  //      openLoopElevatorCommand.name = "ElevatorZeroVoltageOpenLoopCommand"
  //    }
  //
  //    return openLoopElevatorCommand
  //  }
  //
  //  fun homeElevatorCommand(): CommandBase {
  //    val maybeHomeElevatorCommand =
  //      run { io.setOutputVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE) }
  //        .until {
  //          isHomed ||
  //            inputs.elevatorPosition > ElevatorConstants.HOMING_POSITION_THRESHOLD ||
  //            inputs.leaderStatorCurrent > ElevatorConstants.HOMING_STALL_CURRENT
  //        }
  //        .finallyDo {
  //          io.zeroEncoder()
  //          isHomed = true
  //        }
  //    maybeHomeElevatorCommand.name = "ElevatorHomingCommand"
  //    return maybeHomeElevatorCommand
  //  }

  fun updateMech2d() {
    // updating carriage attachment based on height of elevator
    val secondStagePosition =
      MathUtil.clamp(
        inputs.elevatorPosition.inInches, 0.0, ElevatorConstants.FIRST_STAGE_HEIHT.inInches
      )
        .inches

    val carriagePosition =
      MathUtil.clamp(
        inputs.elevatorPosition.inInches,
        secondStagePosition.inInches,
        ElevatorConstants.ELEVATOR_MAX_EXTENSION.inInches
      )
        .inches

    secondStageAttachment.setPosition(
      (
        elevatorAbsoluteXPosition -
          ElevatorConstants.ElevatorStates.fromPositionToRange(secondStagePosition)
        )
        .inInches,
      (
        elevatorAbsoluteYPosition +
          ElevatorConstants.ElevatorStates.fromPositionToHeight(secondStagePosition)
        )
        .inInches
    )
    carriageAttachment.setPosition(
      (
        elevatorAbsoluteXPosition -
          ElevatorConstants.ElevatorStates.fromPositionToRange(carriagePosition)
        )
        .inInches,
      (
        elevatorAbsoluteYPosition +
          ElevatorConstants.ElevatorStates.fromPositionToHeight(carriagePosition)
        )
        .inInches
    )
  }

  companion object {
    enum class ElevatorState {
      UNINITIALIZED,
      TARGETING_POSITION,
      OPEN_LOOP,
      HOME;

      inline fun equivalentToRequest(request: ElevatorRequest): Boolean {
        return ((request is ElevatorRequest.Home) && this == HOME) ||
          (request is ElevatorRequest.OpenLoop && this == OPEN_LOOP) ||
          (request is ElevatorRequest.TargetingPosition && this == TARGETING_POSITION)
      }
    }

    inline fun fromElevatorRequestToState(request: ElevatorRequest): ElevatorState {
      return when (request) {
        is ElevatorRequest.Home -> ElevatorState.HOME
        is ElevatorRequest.OpenLoop -> ElevatorState.OPEN_LOOP
        is ElevatorRequest.TargetingPosition -> ElevatorState.TARGETING_POSITION
      }
    }

    inline fun fromHeightToPosition(height: Length): Length {
      return height / ElevatorConstants.ELEVATOR_ANGLE.sin
    }

    inline fun fromPositionToHeight(position: Length): Length {
      return position * ElevatorConstants.ELEVATOR_ANGLE.sin
    }

    inline fun fromRangeToPosition(range: Length): Length {
      return range / ElevatorConstants.ELEVATOR_ANGLE.cos
    }

    inline fun fromPositionToRange(position: Length): Length {
      return position * ElevatorConstants.ELEVATOR_ANGLE.cos
    }
  }
}
