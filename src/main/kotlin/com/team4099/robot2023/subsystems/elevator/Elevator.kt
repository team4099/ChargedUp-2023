package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.config.constants.MechanismSimConstants.carriageAttachment
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteXPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteYPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.secondStageAttachment
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import java.util.function.Supplier

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()

  // PID and Feedforward Values
  lateinit var elevatorFeedforward: ElevatorFeedforward

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

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFTLIMIT_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFTLIMIT_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION

  var isHomed = false

  // Iterate through all desired states and see if the current position is equivalent to any of the
  // actual positions. If not, return that it's between two positions.
  var desiredHeight = ElevatorConstants.ElevatorStates.MIN_HEIGHT.height

  val isAtCommandedState: Boolean
    get() {
      return (
        (
          inputs.elevatorPosition -
            ElevatorConstants.ElevatorStates.fromHeightToPosition(desiredHeight)
          )
          .absoluteValue < ElevatorConstants.ELEVATOR_TOLERANCE
        )
    }

  val actualElevatorStates =
    hashMapOf<ElevatorConstants.ElevatorStates, LoggedTunableValue<Meter>>()

  // trapezoidal profile stuff
  var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)

  init {

    ElevatorConstants.ElevatorStates.values().forEach {
      actualElevatorStates[it] =
        LoggedTunableValue<Meter>(
          "Elevator/${it.name}", it.height, Pair({ it.inInches }, { it.inches })
        )
    }

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

    Logger.getInstance().recordOutput("Elevator/isAtCommandedState", isAtCommandedState)
    Logger.getInstance()
      .recordOutput(
        "Elevator/elevatorHeightWithRespectToGround",
        ElevatorConstants.ElevatorStates.fromPositionToHeight(inputs.elevatorPosition).inInches
      )
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

  /**
   * Command factory for the elevator idle/hold
   *
   * @return Command which keeps elevator at its current position uses small amount of feedforward
   * to overcome force of gravity
   */
  fun holdElevatorPosition(): Command {
    var positionToHold = inputs.elevatorPosition
    val holdPositionCommand =
      runOnce {
        positionToHold = inputs.elevatorPosition
        Logger.getInstance().recordOutput("Elevator/holdPosition", positionToHold.inInches)
      }
        .andThen(
          run {
            // check if hold position is close to zero
            if (inputs.elevatorPosition.absoluteValue <
              ElevatorConstants.ELEVATOR_TOLERANCE
            ) {
              io.setOutputVoltage(0.0.volts)
            } else {
              io.setPosition(
                positionToHold, elevatorFeedforward.calculate(0.meters.perSecond)
              )
            }
          }
        )

    holdPositionCommand.name = "ElevatorHoldPositionCommand"
    return holdPositionCommand
  }

  /**
   * Inline command to set the elevator to a desired position.
   *
   * @param position The desired position of the elevator in its frame of reference.
   *
   * @return A command that runs the elevator's setPosition function until the profile is finished
   * running.
   */
  fun raiseElevatorHeight(state: ElevatorConstants.ElevatorStates): CommandBase {

    // Constructing our elevator profile in here because its internal values are dependent on the
    // position we want to set the elevator to
    val position = Supplier {
      ElevatorConstants.ElevatorStates.fromHeightToPosition(
        actualElevatorStates[state]?.get() ?: ElevatorConstants.ElevatorStates.MIN_HEIGHT.height
      )
    }

    // Creates a command that runs continuously until the profile is finished. The run function
    // accepts a lambda which indicates what we want to run every iteration.
    val raiseElevatorHeight = generateElevatorMoveCommand(position, 0.0.inches.perSecond)

    raiseElevatorHeight.name = "ElevatorRaiseHeightCommand"
    return raiseElevatorHeight
  }

  fun slamDown(dropDistance: Length): Command {
    val slamDownCommand =
      generateElevatorMoveCommand(
        { inputs.elevatorPosition - dropDistance }, 0.0.inches.perSecond
      )
    slamDownCommand.name = "ElevatorSlamDownCommand"
    return slamDownCommand
  }

  fun generateElevatorMoveCommand(
    targetPosition: Supplier<Length>,
    targetVelocity: LinearVelocity
  ): CommandBase {
    lateinit var elevatorProfile: TrapezoidProfile<Meter>

    // Obtaining a start time for this command so that we can pass it into our profile. This is done
    // here because we need the startTime to represent the time at which the profile began.
    var startTime = Clock.fpgaTime

    val elevatorMovementCommand =
      runOnce {
        startTime = Clock.fpgaTime
        desiredHeight = targetPosition.get()
        elevatorProfile =
          TrapezoidProfile(
            elevatorConstraints,
            TrapezoidProfile.State(targetPosition.get(), targetVelocity),
            TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
          )
        Logger.getInstance().recordOutput("Elevator/isAtSetpoint", false)
      }
        .andThen(
          run {
            setPosition(
              elevatorProfile.calculate(
                Clock.fpgaTime -
                  startTime
              )
            ) // Every loop cycle we have a different profile state
            // we're
            // calculating. Hence, we want to pass in a different Trapezoidal
            // Profile State into the setPosition function.
            Logger.getInstance()
              .recordOutput(
                "/Elevator/isAtSetpoint",
                (inputs.elevatorPosition - targetPosition.get()).absoluteValue <
                  ElevatorConstants.ELEVATOR_TOLERANCE
              )
          }
            .until { // The following lambda creates a race condition that stops the command
              // we
              // created
              // above when the passed in condition is true. In our case it's checking when
              // elevatorProfile is finished based on elapsed time.
              elevatorProfile.isFinished((Clock.fpgaTime - startTime)) ||
                (
                  forwardLimitReached &&
                    targetPosition.get() >
                    ElevatorConstants.ELEVATOR_SOFTLIMIT_EXTENSION ||
                    reverseLimitReached &&
                    targetPosition.get() <
                    ElevatorConstants.ELEVATOR_SOFTLIMIT_RETRACTION
                  )
              // This is the race condition we're passing in.
            }
        )
    return elevatorMovementCommand
  }

  /**
   * Command factory for setting elevator motors to desired voltage
   *
   * @return Command which sets output voltage of both motors and ends if elevator reachs its limit
   */
  fun openLoopControl(voltage: ElectricalPotential): Command {
    val openLoopElevatorCommand =
      run {
        setOutputVoltage(voltage)
        if (voltage > 0.volts) {
          Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopExtend", true)
        } else {
          Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopRetract", true)
        }
      }
        .until {
          (
            forwardOpenLoopLimitReached && voltage > 0.volts ||
              reverseOpenLoopLimitReached && voltage < 0.volts
            )
        }
        .finallyDo {
          setOutputVoltage(0.volts)
          if (voltage > 0.volts) {
            Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopExtend", false)
          } else {
            Logger.getInstance().recordOutput("/ActiveCommands/OpenLoopRetract", false)
          }
        }

    if (voltage > 0.volts) {
      openLoopElevatorCommand.name = "ElevatorExtendOpenLoopCommand"
    } else if (voltage < 0.volts) {
      openLoopElevatorCommand.name = "ElevatorRetractOpenLoopCommand"
    } else {
      openLoopElevatorCommand.name = "ElevatorZeroVoltageOpenLoopCommand"
    }

    return openLoopElevatorCommand
  }

  fun homeElevatorCommand(): Command {
    val maybeHomeElevatorCommand =
      run { io.setOutputVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE) }
        .until {
          isHomed ||
            inputs.elevatorPosition > ElevatorConstants.HOMING_POSITION_THRESHOLD ||
            inputs.leaderStatorCurrent > ElevatorConstants.HOMING_STALL_CURRENT
        }
        .finallyDo {
          io.zeroEncoder()
          isHomed = true
        }
    maybeHomeElevatorCommand.name = "ElevatorHomingCommand"
    return maybeHomeElevatorCommand
  }

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
}
