package com.team4099.lib.controller

import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.AccelerationFeedforward
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.LinearGravityFeedforward
import com.team4099.lib.units.derived.StaticFeedforward
import com.team4099.lib.units.derived.VelocityFeedforward
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import com.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import com.team4099.lib.units.derived.perMeterPerSecondPerSecond
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.controller.ElevatorFeedforward as WPIElevatorFeedforward

class ElevatorFeedforward(
  kS: StaticFeedforward,
  kG: LinearGravityFeedforward,
  kV: VelocityFeedforward<Meter>,
  kA: AccelerationFeedforward<Meter>
) {
  private val feedforward =
    WPIElevatorFeedforward(
      kS.inVolts, kG.inVolts, kV.inVoltsPerMeterPerSecond, kA.inVoltsPerMeterPerSecondPerSecond
    )

  constructor(
    kS: StaticFeedforward,
    kG: LinearGravityFeedforward,
    kV: VelocityFeedforward<Meter>
  ) : this(kS, kG, kV, 0.0.volts.perMeterPerSecondPerSecond)

  fun calculate(velocity: LinearVelocity, acceleration: LinearAcceleration): ElectricalPotential {
    return feedforward.calculate(
      velocity.inMetersPerSecond, acceleration.inMetersPerSecondPerSecond
    )
      .volts
  }

  fun calculate(velocity: LinearVelocity): ElectricalPotential {
    return feedforward.calculate(velocity.inMetersPerSecond).volts
  }

  fun maxAchievableVelocity(
    maxVoltage: ElectricalPotential,
    acceleration: LinearAcceleration
  ): LinearVelocity {
    return feedforward.maxAchievableVelocity(
      maxVoltage.inVolts, acceleration.inMetersPerSecondPerSecond
    )
      .meters
      .perSecond
  }

  fun minAchievableVelocity(
    maxVoltage: ElectricalPotential,
    acceleration: LinearAcceleration
  ): LinearVelocity {
    return feedforward.minAchievableVelocity(
      maxVoltage.inVolts, acceleration.inMetersPerSecondPerSecond
    )
      .meters
      .perSecond
  }

  fun maxAchievableAcceleration(
    maxVoltage: ElectricalPotential,
    velocity: LinearVelocity
  ): LinearAcceleration {
    return feedforward.maxAchievableAcceleration(maxVoltage.inVolts, velocity.inMetersPerSecond)
      .meters
      .perSecond
      .perSecond
  }

  fun minAchievableAcceleration(
    maxVoltage: ElectricalPotential,
    velocity: LinearVelocity
  ): LinearAcceleration {
    return feedforward.minAchievableAcceleration(maxVoltage.inVolts, velocity.inMetersPerSecond)
      .meters
      .perSecond
      .perSecond
  }
}
