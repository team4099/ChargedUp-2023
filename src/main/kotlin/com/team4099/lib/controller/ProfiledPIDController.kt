package com.team4099.lib.controller

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.base.Second
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.DerivativeGain
import com.team4099.lib.units.derived.IntegralGain
import com.team4099.lib.units.derived.ProportionalGain
import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.math.controller.ProfiledPIDController as WPIProfiledPIDController

class ProfiledPIDController<E : UnitKey, O : UnitKey>(
  proportionalGain: ProportionalGain<E, O>,
  integralGain: IntegralGain<E, O>,
  derivativeGain: DerivativeGain<E, O>,
  constraints: TrapezoidProfile.Constraints<E>,
  period: Time
) {
  val wpiPidController =
    WPIProfiledPIDController(
      proportionalGain.value,
      integralGain.value,
      derivativeGain.value,
      constraints.wpiConstraints,
      period.inSeconds
    )

  var proportionalGain: ProportionalGain<E, O>
    get() = ProportionalGain(wpiPidController.p)
    set(value) {
      wpiPidController.p = value.value
    }

  val setpoint: TrapezoidProfile.State<E>
    get() = TrapezoidProfile.State(wpiPidController.setpoint)

  val atSetpoint: Boolean
    get() = wpiPidController.atSetpoint()

  var integralGain: IntegralGain<E, O>
    get() = IntegralGain(wpiPidController.i)
    set(value) {
      wpiPidController.i = value.value
    }

  var derivativeGain: DerivativeGain<E, O>
    get() = DerivativeGain(wpiPidController.d)
    set(value) {
      wpiPidController.d = value.value
    }

  val period: Time
    get() = wpiPidController.period.seconds

  var errorTolerance: Value<E>
    get() = Value(wpiPidController.positionTolerance)
    set(value) {
      wpiPidController.setTolerance(value.value)
    }

  var errorDerivativeTolerance: Value<Fraction<E, Second>>
    get() = Value(wpiPidController.velocityTolerance)
    set(value) {
      wpiPidController.setTolerance(wpiPidController.positionTolerance, value.value)
    }

  var goal: TrapezoidProfile.State<E>
    get() = TrapezoidProfile.State(wpiPidController.goal)
    set(value) {
      wpiPidController.goal = value.wpiState
    }

  val atGoal: Boolean
    get() = wpiPidController.atGoal()

  val error: Value<E>
    get() = Value(wpiPidController.positionError)

  val errorDerivative: Value<Fraction<E, Second>>
    get() = Value(wpiPidController.velocityError)

  // TODO: Fix this to not rely on robot code
  constructor(
    proportionalGain: ProportionalGain<E, O>,
    integralGain: IntegralGain<E, O>,
    derivativeGain: DerivativeGain<E, O>,
    constraints: TrapezoidProfile.Constraints<E>
  ) : this(
    proportionalGain,
    integralGain,
    derivativeGain,
    constraints,
    Constants.Universal.LOOP_PERIOD_TIME
  )

  fun setGoal(goal: Value<E>) {
    wpiPidController.setGoal(goal.value)
  }

  fun calculate(measurement: Value<E>, goal: Value<E>): Value<O> {
    return Value(wpiPidController.calculate(measurement.value, goal.value))
  }

  fun calculate(measurement: Value<E>): Value<O> {
    return Value(wpiPidController.calculate(measurement.value))
  }

  fun calculate(measurement: Value<E>, goal: TrapezoidProfile.State<E>): Value<O> {
    return Value(wpiPidController.calculate(measurement.value, goal.wpiState))
  }

  fun calculate(
    measurement: Value<E>,
    goal: TrapezoidProfile.State<E>,
    constraints: TrapezoidProfile.Constraints<E>
  ): Value<O> {
    return Value(
      wpiPidController.calculate(measurement.value, goal.wpiState, constraints.wpiConstraints)
    )
  }

  fun enableContinuousInput(minimumInput: Value<E>, maximumInput: Value<E>) {
    wpiPidController.enableContinuousInput(minimumInput.value, maximumInput.value)
  }

  fun disableContinuousInput() {
    wpiPidController.disableContinuousInput()
  }

  fun setIntegratorRange(
    minimumIntegral: Value<Product<E, Second>>,
    maximumIntegral: Value<Product<E, Second>>
  ) {
    wpiPidController.setIntegratorRange(minimumIntegral.value, maximumIntegral.value)
  }

  fun reset(measurement: TrapezoidProfile.State<E>) {
    wpiPidController.reset(measurement.wpiState)
  }

  fun reset(measuredPosition: Value<E>) {
    wpiPidController.reset(measuredPosition.value)
  }

  fun setPID(kP: ProportionalGain<E, O>, kI: IntegralGain<E, O>, kD: DerivativeGain<E, O>) {
    proportionalGain = kP
    integralGain = kI
    derivativeGain = kD
  }

  fun setConstraints(constraints: TrapezoidProfile.Constraints<E>) {
    wpiPidController.setConstraints(constraints.wpiConstraints)
  }
}
