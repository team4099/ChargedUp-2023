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
import edu.wpi.first.math.controller.PIDController as WPIPidController

class PIDController<E : UnitKey, O : UnitKey> {
  val wpiPidController: WPIPidController

  var proportionalGain: ProportionalGain<E, O>
    get() = ProportionalGain(wpiPidController.p)
    set(value) {
      wpiPidController.p = value.value
    }

  val isAtSetpoint: Boolean
    get() = wpiPidController.atSetpoint()

  val isContinuousInputEnabled: Boolean
    get() = wpiPidController.isContinuousInputEnabled

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

  val error: Value<E>
    get() = Value(wpiPidController.positionError)

  val errorDerivative: Value<Fraction<E, Second>>
    get() = Value(wpiPidController.velocityError)

  constructor(
    proportionalGain: ProportionalGain<E, O>,
    integralGain: IntegralGain<E, O>,
    derivativeGain: DerivativeGain<E, O>,
  ) {
    wpiPidController =
      WPIPidController(proportionalGain.value, integralGain.value, derivativeGain.value)
  }

  constructor(
    proportionalGain: ProportionalGain<E, O>,
    integralGain: IntegralGain<E, O>,
    derivativeGain: DerivativeGain<E, O>,
    period: Time
  ) {
    wpiPidController =
      WPIPidController(
        proportionalGain.value, integralGain.value, derivativeGain.value, period.inSeconds
      )
  }

  fun calculate(measurement: Value<E>, setpoint: Value<E>): Value<O> {
    return Value(wpiPidController.calculate(measurement.value, setpoint.value))
  }

  fun calculate(measurement: Value<E>): Value<O> {
    return Value(wpiPidController.calculate(measurement.value))
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

  fun reset() {
    wpiPidController.reset()
  }

  fun setPID(kP: ProportionalGain<E, O>, kI: IntegralGain<E, O>, kD: DerivativeGain<E, O>){
    proportionalGain = kP
    integralGain = kI
    derivativeGain = kD
  }
}
