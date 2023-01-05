package com.team4099.lib.controller

import com.team4099.lib.units.Acceleration
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.Velocity
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.derived.AccelerationFeedforward
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.StaticFeedforward
import com.team4099.lib.units.derived.VelocityFeedforward
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.volts
import edu.wpi.first.math.controller.SimpleMotorFeedforward as WPISimpleFeedforward

class SimpleMotorFeedforward<U : UnitKey>(
  val kS: StaticFeedforward,
  val kV: VelocityFeedforward<U>,
  val kA: AccelerationFeedforward<U>
) {
  private val feedforward: WPISimpleFeedforward =
    WPISimpleFeedforward(kS.inVolts, kV.value, kA.value)

  constructor(
    kS: ElectricalPotential,
    kV: VelocityFeedforward<U>
  ) : this(kS, kV, AccelerationFeedforward(0.0))

  fun calculate(
    velocity: Value<Velocity<U>>,
    acceleration: Value<Acceleration<U>>
  ): ElectricalPotential {
    return feedforward.calculate(velocity.value, acceleration.value).volts
  }

  fun calculate(
    currentVelocitySetpoint: Value<Velocity<U>>,
    nextVelocitySetpoint: Value<Velocity<U>>,
    dT: Time
  ): ElectricalPotential {
    return feedforward.calculate(
      currentVelocitySetpoint.value, nextVelocitySetpoint.value, dT.inSeconds
    )
      .volts
  }

  fun calculate(velocity: Value<Velocity<U>>): ElectricalPotential {
    return feedforward.calculate(velocity.value).volts
  }

  fun maxAchievableVelocity(
    maxVoltage: ElectricalPotential,
    acceleration: Value<Acceleration<U>>
  ): Value<Velocity<U>> {
    return Value(feedforward.maxAchievableVelocity(maxVoltage.inVolts, acceleration.value))
  }

  fun minAchievableVelocity(
    maxVoltage: ElectricalPotential,
    acceleration: Value<Acceleration<U>>
  ): Value<Velocity<U>> {
    return Value(feedforward.minAchievableVelocity(maxVoltage.inVolts, acceleration.value))
  }

  fun maxAchievableAcceleration(
    maxVoltage: ElectricalPotential,
    velocity: Value<Velocity<U>>
  ): Value<Acceleration<U>> {
    return Value(feedforward.maxAchievableAcceleration(maxVoltage.inVolts, velocity.value))
  }

  fun minAchievableAcceleration(
    maxVoltage: ElectricalPotential,
    velocity: Value<Velocity<U>>
  ): Value<Acceleration<U>> {
    return Value(feedforward.minAchievableAcceleration(maxVoltage.inVolts, velocity.value))
  }
}
