package com.team4099.lib.units

import com.ctre.phoenix.motorcontrol.can.BaseTalon
import com.revrobotics.CANSparkMax
import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.minutes
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.DerivativeGain
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.IntegralGain
import com.team4099.lib.units.derived.ProportionalGain
import com.team4099.lib.units.derived.Radian
import com.team4099.lib.units.derived.VelocityFeedforward
import com.team4099.lib.units.derived.Volt
import com.team4099.lib.units.derived.inRotations
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.inVoltsPerMeter
import com.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import com.team4099.lib.units.derived.inVoltsPerMeterSeconds
import com.team4099.lib.units.derived.inVoltsPerMeters
import com.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import com.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import com.team4099.lib.units.derived.inVoltsPerRadian
import com.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import com.team4099.lib.units.derived.inVoltsPerRadianSeconds
import com.team4099.lib.units.derived.inVoltsPerRadians
import com.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import com.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.rotations
import kotlin.math.PI

enum class Timescale(val velocity: Time, val acceleration: Time) {
  REV_NEO(1.minutes, 1.seconds),
  CTRE(100.milli.seconds, 1.seconds),
}

interface MechanismSensor<U : UnitKey> {
  val position: Value<U>
  val velocity: Value<Velocity<U>>

  fun positionToRawUnits(position: Value<U>): Double
  fun velocityToRawUnits(velocity: Value<Velocity<U>>): Double
  fun accelerationToRawUnits(acceleration: Value<Acceleration<U>>): Double

  fun proportionalPositionGainToRawUnits(
    proportionalGain: ProportionalGain<U, Volt>,
  ): Double
  fun integralPositionGainToRawUnits(
    integralGain: IntegralGain<U, Volt>,
  ): Double
  fun derivativePositionGainToRawUnits(
    derivativeGain: DerivativeGain<U, Volt>,
  ): Double

  fun proportionalVelocityGainToRawUnits(
    proportionalGain: ProportionalGain<Velocity<U>, Volt>,
  ): Double
  fun integralVelocityGainToRawUnits(
    integralGain: IntegralGain<Velocity<U>, Volt>,
  ): Double
  fun derivativeVelocityGainToRawUnits(
    derivativeGain: DerivativeGain<Velocity<U>, Volt>,
  ): Double

  fun velocityFeedforwardToRawUnits(velocityFeedforward: VelocityFeedforward<U>): Double
}

class LinearMechanismSensor(
  private val diameter: Length,
  private val ratio: Double,
  private val timescale: Timescale,
  private val fullPowerThrottle: Double,
  private val compensationVoltage: ElectricalPotential,
  val getRawVelocity: () -> Double,
  val getRawPosition: () -> Double
) : MechanismSensor<Meter> {
  override val position: Length
    get() = (diameter * PI) * getRawPosition() * ratio

  override val velocity: Value<Velocity<Meter>>
    get() {
      val linearUnscaledVelocity = diameter.inMeters * getRawVelocity() * ratio * PI
      return (linearUnscaledVelocity / timescale.velocity.inSeconds).meters.perSecond
    }

  override fun positionToRawUnits(position: Value<Meter>): Double = position / diameter / ratio / PI

  override fun velocityToRawUnits(velocity: Value<Velocity<Meter>>): Double {
    val linearUnscaledVelocity = velocity.inMetersPerSecond * timescale.velocity.inSeconds
    return linearUnscaledVelocity / diameter.inMeters / ratio / PI
  }

  override fun accelerationToRawUnits(acceleration: Value<Acceleration<Meter>>): Double {
    val linearUnscaledVelocity =
      velocity.inMetersPerSecond * timescale.velocity.inSeconds * timescale.acceleration.inSeconds
    return linearUnscaledVelocity / diameter.inMeters / ratio / PI
  }

  override fun proportionalPositionGainToRawUnits(
    proportionalGain: ProportionalGain<Meter, Volt>
  ): Double {
    return (proportionalGain.inVoltsPerMeter / (positionToRawUnits(1.meters))) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun integralPositionGainToRawUnits(
    integralGain: IntegralGain<Meter, Volt>,
  ): Double {
    return (
      integralGain.inVoltsPerMeterSeconds /
        (positionToRawUnits(1.meters) * timescale.velocity.inSeconds)
      ) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun derivativePositionGainToRawUnits(
    derivativeGain: DerivativeGain<Meter, Volt>,
  ): Double {
    return (
      derivativeGain.inVoltsPerMeterPerSecond * timescale.velocity.inSeconds /
        positionToRawUnits(1.meters)
      ) / compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun proportionalVelocityGainToRawUnits(
    proportionalGain: ProportionalGain<Velocity<Meter>, Volt>
  ): Double {
    return (proportionalGain.inVoltsPerMetersPerSecond / (velocityToRawUnits(1.meters.perSecond))) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun integralVelocityGainToRawUnits(
    integralGain: IntegralGain<Velocity<Meter>, Volt>,
  ): Double {
    return (
      integralGain.inVoltsPerMeters /
        (velocityToRawUnits(1.meters.perSecond) * timescale.velocity.inSeconds)
      ) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun derivativeVelocityGainToRawUnits(
    derivativeGain: DerivativeGain<Velocity<Meter>, Volt>,
  ): Double {
    return (
      derivativeGain.inVoltsPerMetersPerSecondPerSecond * timescale.velocity.inSeconds /
        velocityToRawUnits(1.meters.perSecond)
      ) / compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun velocityFeedforwardToRawUnits(
    velocityFeedforward: VelocityFeedforward<Meter>
  ): Double {
    return velocityFeedforward.value * velocityToRawUnits(1.0.meters.perSecond) /
      compensationVoltage.inVolts * fullPowerThrottle
  }
}

class AngularMechanismSensor(
  private val ratio: Double,
  private val timescale: Timescale,
  private val fullPowerThrottle: Double,
  private val compensationVoltage: ElectricalPotential,
  val getRawVelocity: () -> Double,
  val getRawPosition: () -> Double
) : MechanismSensor<Radian> {
  override val position: Value<Radian>
    get() = (getRawPosition() * ratio).rotations

  override val velocity: Value<Velocity<Radian>>
    get() = (getRawVelocity() * ratio / timescale.velocity.inSeconds).rotations.perSecond

  override fun positionToRawUnits(position: Value<Radian>): Double = position.inRotations / ratio

  override fun velocityToRawUnits(velocity: Value<Velocity<Radian>>): Double =
    (velocity.inRotationsPerSecond * timescale.velocity.inSeconds) / ratio

  override fun accelerationToRawUnits(acceleration: Value<Acceleration<Radian>>): Double =
    (
      acceleration.inRotationsPerSecondPerSecond *
        timescale.velocity.inSeconds *
        timescale.acceleration.inSeconds
      ) / ratio

  override fun proportionalPositionGainToRawUnits(
    proportionalGain: ProportionalGain<Radian, Volt>
  ): Double {
    return (proportionalGain.inVoltsPerRadian / (positionToRawUnits(1.radians))) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun integralPositionGainToRawUnits(integralGain: IntegralGain<Radian, Volt>): Double {
    return (
      integralGain.inVoltsPerRadianSeconds /
        (positionToRawUnits(1.radians) * timescale.velocity.inSeconds)
      ) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun derivativePositionGainToRawUnits(
    derivativeGain: DerivativeGain<Radian, Volt>
  ): Double {
    return (
      derivativeGain.inVoltsPerRadianPerSecond * timescale.velocity.inSeconds /
        positionToRawUnits(1.radians)
      ) / compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun proportionalVelocityGainToRawUnits(
    proportionalGain: ProportionalGain<Velocity<Radian>, Volt>
  ): Double {
    return (
      proportionalGain.inVoltsPerRadiansPerSecond /
        (velocityToRawUnits(1.radians.perSecond))
      ) / compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun integralVelocityGainToRawUnits(
    integralGain: IntegralGain<Velocity<Radian>, Volt>
  ): Double {
    return (
      integralGain.inVoltsPerRadians /
        (velocityToRawUnits(1.radians.perSecond) * timescale.velocity.inSeconds)
      ) /
      compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun derivativeVelocityGainToRawUnits(
    derivativeGain: DerivativeGain<Velocity<Radian>, Volt>
  ): Double {
    return (
      derivativeGain.inVoltsPerRadiansPerSecondPerSecond * timescale.velocity.inSeconds /
        velocityToRawUnits(1.radians.perSecond)
      ) / compensationVoltage.inVolts * fullPowerThrottle
  }

  override fun velocityFeedforwardToRawUnits(
    velocityFeedforward: VelocityFeedforward<Radian>
  ): Double {
    return velocityFeedforward.value * velocityToRawUnits(1.0.radians.perSecond) /
      compensationVoltage.inVolts * fullPowerThrottle
  }
}

fun ctreAngularMechanismSensor(
  controller: BaseTalon,
  sensorCpr: Int,
  ratio: Double,
  compensationVoltage: ElectricalPotential
): AngularMechanismSensor {
  return AngularMechanismSensor(
    ratio / sensorCpr,
    Timescale.CTRE,
    1023.0,
    compensationVoltage,
    { controller.selectedSensorVelocity.toDouble() },
    { controller.selectedSensorPosition.toDouble() }
  )
}

fun ctreLinearMechanismSensor(
  controller: BaseTalon,
  sensorCpr: Int,
  ratio: Double,
  diameter: Length,
  compensationVoltage: ElectricalPotential
): LinearMechanismSensor {
  return LinearMechanismSensor(
    diameter,
    ratio / sensorCpr,
    Timescale.CTRE,
    1023.0,
    compensationVoltage,
    { controller.selectedSensorVelocity.toDouble() },
    { controller.selectedSensorPosition.toDouble() }
  )
}

fun sparkMaxAngularMechanismSensor(
  controller: CANSparkMax,
  ratio: Double,
  compensationVoltage: ElectricalPotential
): AngularMechanismSensor {
  val encoder = controller.encoder

  return AngularMechanismSensor(
    ratio,
    Timescale.REV_NEO,
    1.0,
    compensationVoltage,
    { encoder.velocity },
    { encoder.position }
  )
}

fun sparkMaxLinearMechanismSensor(
  controller: CANSparkMax,
  ratio: Double,
  diameter: Length,
  compensationVoltage: ElectricalPotential
): LinearMechanismSensor {
  val encoder = controller.encoder

  return LinearMechanismSensor(
    diameter,
    ratio,
    Timescale.REV_NEO,
    1.0,
    compensationVoltage,
    { encoder.velocity },
    { encoder.position }
  )
}
