package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()
  val elevatorFeedForward =
    ElevatorFeedforward(
      ElevatorConstants.ELEVATOR_KS,
      ElevatorConstants.ELEVATOR_KG,
      ElevatorConstants.ELEVATOR_KV,
      ElevatorConstants.ELEVATOR_KA
    )

  private val kP =
    LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerMeter }, { it.volts.perMeter }))
  private val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerMeterSeconds }, { it.volts.perMeterSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerMeterPerSecond }, { it.volts / 1.0.meters.perSecond })
    )

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_MAX_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_MAX_RETRACTION

  var desiredState = ElevatorConstants.DesiredElevatorStates.MIN_HEIGHT

  val desiredPosition: Length
    get() {
      return desiredState.height / ElevatorConstants.ELEVATOR_ANGLE.sin
    }

  // Iterate through all desired states and see if the current position is equivalent to any of the
  // actual positions. If not, return that it's between two positions.
  val currentState: ElevatorConstants.ActualElevatorStates
    get() {
      for (state in ElevatorConstants.DesiredElevatorStates.values()) {
        if ((state.height - inputs.elevatorPosition).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) {
          return ElevatorConstants.ActualElevatorStates.fromDesiredState(state)
        }
      }
      // TODO figure out if we'll ever need to know if we're between two states
      return ElevatorConstants.ActualElevatorStates.BETWEEN_TWO_STATES
    }

  var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(ElevatorConstants.REAL_KP)
      kI.initDefault(ElevatorConstants.REAL_KI)
      kD.initDefault(ElevatorConstants.REAL_KD)
    } else {
      kP.initDefault(ElevatorConstants.SIM_KP)
      kI.initDefault(ElevatorConstants.SIM_KI)
      kD.initDefault(ElevatorConstants.SIM_KD)
    }

    io.configPID(kP.get(), kI.get(), kD.get())
  }

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  fun setOpenLoop(percentOutput: Double) {
    if ((forwardLimitReached && percentOutput > 0) || (reverseLimitReached && percentOutput < 0)) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }

    Logger.getInstance().recordOutput("Elevator/percentOutput", percentOutput)
  }

  fun setPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAccel =
      ((setpoint.velocity - elevatorSetpoint.velocity) / (Constants.Universal.LOOP_PERIOD_TIME))

    elevatorSetpoint = setpoint

    var feedforward = elevatorFeedForward.calculate(setpoint.velocity, elevatorAccel)

    io.setPosition(setpoint.position, feedforward)

    Logger.getInstance().recordOutput("Elevator/targetPosition", setpoint.position.inInches)
    Logger.getInstance().recordOutput("Elevator/targetVel", setpoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorAcceleration", elevatorAccel.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Elevator/elevatorFeedFoward", feedforward.inVolts)
  }

  fun zeroEncoder() {
    io.zeroEncoder()
  }

  fun holdElevatorPosition(): Command {
    return run {
      io.setPosition(
        inputs.elevatorPosition, elevatorFeedForward.calculate(0.0001.meters.perSecond)
      )
    }
  }

  /**
   * Inline command to set the elevator to a desired position.
   *
   * @param position The desired position of the elevator in its frame of reference.
   *
   * @return A command that runs the elevator's setPosition function until the profile is finished
   * running.
   */
  fun setElevatorPosition(position: Length): Command {

    // Constructing our elevator profile in here because its internal values are dependent on the
    // position we want to set the elevator to
    val elevatorProfile =
      TrapezoidProfile(
        elevatorConstraints,
        TrapezoidProfile.State(position, 0.0.meters / 1.0.seconds),
        TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
      )

    // Obtaining a start time for this command so that we can pass it into our profile. This is done
    // here because we need the startTime to represent the time at which the profile began.
    val startTime = Clock.fpgaTime

    // Creates a command that runs continuously until the profile is finished. The run function
    // accepts a lambda which indicates what we want to run every iteration.
    return run {
      setPosition(
        elevatorProfile.calculate(
          Clock.fpgaTime -
            startTime
        )
      ) // Every loop cycle we have a different profile state we're
      // calculating. Hence, we want to pass in a different Trapezoidal
      // Profile State into the setPosition function.
    }
      .until { // The following lambda creates a race condition that stops the command we created
        // above when the passed in condition is true. In our case it's checking when
        // elevatorProfile is finished based on elapsed time.
        elevatorProfile.isFinished(
          (Clock.fpgaTime - startTime)
        ) // This is the race condition we're passing in.
      }
  }

  fun openLoopControl(percentOutput: Double): Command {
    return run { setOpenLoop(percentOutput) }.finallyDo { setOpenLoop(0.0) }
  }
}
