package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Unitless
import com.team4099.lib.units.Value
import com.team4099.lib.units.Velocity
import com.team4099.lib.units.base.Ampere
import com.team4099.lib.units.base.METERS_PER_FOOT
import com.team4099.lib.units.base.METERS_PER_INCH
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.SECONDS_PER_MINUTE
import com.team4099.lib.units.base.Second

typealias ProportionalGain<E, O> = Value<Fraction<O, E>>

typealias IntegralGain<E, O> = Value<Fraction<O, Product<E, Second>>>

typealias DerivativeGain<E, O> = Value<Fraction<O, Fraction<E, Second>>>

val Double.metersPerMeter
  get() = Value<Fraction<Meter, Meter>>(this)

val Double.metersPerSecondPerMetersPerSecond
  get() = Value<Fraction<Fraction<Meter, Second>, Fraction<Meter, Second>>>(this)

val Value<Unitless>.metersPerMeter
  get() = Value<Fraction<Meter, Meter>>(value)

val Value<Unitless>.metersPerSecondPerMeterPerSecond
  get() = Value<Fraction<Fraction<Meter, Second>, Fraction<Meter, Second>>>(value)

val <K : UnitKey> Value<K>.perMeter
  get() = Value<Fraction<K, Meter>>(value)

val <K : UnitKey> Value<K>.perInch
  get() = perMeter / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFoot
  get() = perMeter / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadian
  get() = Value<Fraction<K, Radian>>(value)

val <K : UnitKey> Value<K>.perDegree
  get() = perRadian / RADIANS_PER_DEGREES

val <K : UnitKey> Value<K>.perRotation
  get() = perRadian / RADIANS_PER_ROTATION

val <K : UnitKey> Value<K>.perMeterSeconds
  get() = Value<Fraction<K, Product<Meter, Second>>>(value)

val <K : UnitKey> Value<K>.perInchSeconds
  get() = perMeterSeconds / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFootSeconds
  get() = perMeterSeconds / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadianSeconds
  get() = Value<Fraction<K, Product<Radian, Second>>>(value)

val <K : UnitKey> Value<K>.perDegreeSeconds
  get() = perRadianSeconds / RADIANS_PER_DEGREES

val <K : UnitKey> Value<K>.perMeterPerSecond
  get() = Value<Fraction<K, Fraction<Meter, Second>>>(value)

val <K : UnitKey> Value<K>.perInchPerSecond
  get() = perMeterPerSecond / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFootPerSecond
  get() = perMeterPerSecond / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadianPerSecond
  get() = Value<Fraction<K, Fraction<Radian, Second>>>(value)

val <K : UnitKey> Value<K>.perDegreePerSecond
  get() = perRadianPerSecond / RADIANS_PER_DEGREES

val ProportionalGain<Meter, Volt>.inVoltsPerMeter: Double
  get() = value

val ProportionalGain<Meter, Volt>.inVoltsPerInch: Double
  get() = inVoltsPerMeter / METERS_PER_INCH

val ProportionalGain<Meter, Volt>.inVoltsPerFoot: Double
  get() = inVoltsPerMeter / METERS_PER_FOOT

val ProportionalGain<Radian, Volt>.inVoltsPerRadian: Double
  get() = value

val ProportionalGain<Radian, Volt>.inVoltsPerDegrees: Double
  get() = inVoltsPerRadian / RADIANS_PER_DEGREES

val ProportionalGain<Radian, Volt>.inVoltsPerRotation: Double
  get() = inVoltsPerRadian / RADIANS_PER_ROTATION

val ProportionalGain<Velocity<Meter>, Volt>.inVoltsPerMetersPerSecond: Double
  get() = value

val ProportionalGain<Velocity<Radian>, Volt>.inVoltsPerRadiansPerSecond: Double
  get() = value

val IntegralGain<Meter, Volt>.inVoltsPerMeterSeconds: Double
  get() = value

val IntegralGain<Meter, Volt>.inVoltsPerInchSeconds: Double
  get() = inVoltsPerMeterSeconds / METERS_PER_INCH

val IntegralGain<Meter, Volt>.inVoltsPerFootSeconds: Double
  get() = inVoltsPerMeterSeconds / METERS_PER_FOOT

val IntegralGain<Velocity<Meter>, Volt>.inVoltsPerMeters: Double
  get() = value

val IntegralGain<Radian, Volt>.inVoltsPerRadianSeconds: Double
  get() = value

val IntegralGain<Radian, Volt>.inVoltsPerDegreeSeconds: Double
  get() = inVoltsPerRadianSeconds / RADIANS_PER_DEGREES

val IntegralGain<Radian, Volt>.inVoltsPerRotationsMinutes: Double
  get() = inVoltsPerRadianSeconds * SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

val IntegralGain<Velocity<Radian>, Volt>.inVoltsPerRadians: Double
  get() = value

val DerivativeGain<Meter, Volt>.inVoltsPerMeterPerSecond: Double
  get() = value

val DerivativeGain<Meter, Volt>.inVoltsPerInchPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_INCH

val DerivativeGain<Meter, Volt>.inVoltsPerFootPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_FOOT

val DerivativeGain<Velocity<Meter>, Volt>.inVoltsPerMetersPerSecondPerSecond: Double
  get() = value

val DerivativeGain<Radian, Volt>.inVoltsPerRadianPerSecond: Double
  get() = value

val DerivativeGain<Radian, Volt>.inVoltsPerDegreePerSecond: Double
  get() = inVoltsPerRadianPerSecond / RADIANS_PER_DEGREES

val DerivativeGain<Radian, Volt>.inVoltsPerRotationsPerMinute: Double
  get() = inVoltsPerRadianPerSecond / SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

val DerivativeGain<Velocity<Radian>, Volt>.inVoltsPerRadiansPerSecondPerSecond: Double
  get() = value

val ProportionalGain<Meter, Ampere>.inAmpsPerMeter: Double
  get() = value

val ProportionalGain<Meter, Ampere>.inAmpsPerInch: Double
  get() = inAmpsPerMeter / METERS_PER_INCH

val ProportionalGain<Meter, Ampere>.inAmpsPerFoot: Double
  get() = inAmpsPerMeter / METERS_PER_FOOT

val ProportionalGain<Radian, Ampere>.inAmpsPerRadian: Double
  get() = value

val ProportionalGain<Radian, Ampere>.inAmpsPerDegrees: Double
  get() = inAmpsPerRadian / RADIANS_PER_DEGREES

val ProportionalGain<Radian, Ampere>.inAmpsPerRotation: Double
  get() = inAmpsPerRadian / RADIANS_PER_ROTATION

val IntegralGain<Meter, Ampere>.inAmpsPerMeterSeconds: Double
  get() = value

val IntegralGain<Meter, Ampere>.inAmpsPerInchSeconds: Double
  get() = inAmpsPerMeterSeconds / METERS_PER_INCH

val IntegralGain<Meter, Ampere>.inAmpsPerFootSeconds: Double
  get() = inAmpsPerMeterSeconds / METERS_PER_FOOT

val IntegralGain<Radian, Ampere>.inAmpsPerRadianSeconds: Double
  get() = value

val IntegralGain<Radian, Ampere>.inAmpsPerDegreeSeconds: Double
  get() = inAmpsPerRadianSeconds / RADIANS_PER_DEGREES

val IntegralGain<Radian, Ampere>.inAmpsPerRotationsMinutes: Double
  get() = inAmpsPerRadianSeconds * SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

val DerivativeGain<Meter, Ampere>.inAmpsPerMeterPerSecond: Double
  get() = value

val DerivativeGain<Meter, Ampere>.inAmpsPerInchPerSecond: Double
  get() = inAmpsPerMeterPerSecond / METERS_PER_INCH

val DerivativeGain<Meter, Ampere>.inAmpsPerFootPerSecond: Double
  get() = inAmpsPerMeterPerSecond / METERS_PER_FOOT

val DerivativeGain<Radian, Ampere>.inAmpsPerRadianPerSecond: Double
  get() = value

val DerivativeGain<Radian, Ampere>.inAmpsPerDegreePerSecond: Double
  get() = inAmpsPerRadianPerSecond / RADIANS_PER_DEGREES

val DerivativeGain<Radian, Ampere>.inAmpsPerRotationsPerMinute: Double
  get() = inAmpsPerRadianPerSecond / SECONDS_PER_MINUTE / RADIANS_PER_ROTATION
