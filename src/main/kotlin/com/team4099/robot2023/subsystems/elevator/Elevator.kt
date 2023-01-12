package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecondPerSecond
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
    get() = inputs.elevatorPosition > ElevatorConstants.ELEVATOR_MAX_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition < ElevatorConstants.ELEVATOR_MAX_RETRACTION

  var desiredState = ElevatorConstants.DesiredElevatorStates.MIN_HEIGHT

  // Iterate through all desired states and see if the current position is equivalent to any of the
  // actual positions. If not, return that it's between two positions.
  val currentState: ElevatorConstants.ActualElevatorStates
    get() {
      for (desiredState in ElevatorConstants.DesiredElevatorStates.values()) {
        if ((desiredState.height - inputs.elevatorPosition).absoluteValue <=
          ElevatorConstants.ELEVATOR_TOLERANCE
        ) {
          return ElevatorConstants.ActualElevatorStates.fromDesiredState(desiredState)
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
  }

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  fun setOpenLoop(percentOutput: Double) {
    io.setOpenLoop(percentOutput)

    if ((forwardLimitReached || reverseLimitReached) && percentOutput != 0.0) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }

    Logger.getInstance().recordOutput("percentOutput", percentOutput)
  }

  fun setPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAccel =
      ((setpoint.velocity - elevatorSetpoint.velocity) / (Constants.Universal.LOOP_PERIOD_TIME))
    elevatorSetpoint = setpoint

    var feedforward = elevatorFeedForward.calculate(setpoint.velocity, elevatorAccel)

    io.setPosition(setpoint.position, feedforward)

    Logger.getInstance().recordOutput("targetPosition", setpoint.position.inMeters)
    Logger.getInstance()
      .recordOutput("elevatorAcceleration", elevatorAccel.inMetersPerSecondPerSecond)
    Logger.getInstance().recordOutput("elevatorFeedFoward", feedforward.inVolts)
  }

  fun zeroEncoder() {
    io.zeroEncoder()
  }
}
