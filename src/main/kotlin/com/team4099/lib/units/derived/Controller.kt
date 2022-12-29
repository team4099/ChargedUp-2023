package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.base.Ampere
import com.team4099.lib.units.base.METERS_PER_FOOT
import com.team4099.lib.units.base.METERS_PER_INCH
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.SECONDS_PER_MINUTE
import com.team4099.lib.units.base.Second

typealias ProportionalGain<E, O> = Value<Fraction<O, E>>

typealias IntegralGain<E, O> = Value<Fraction<O, Product<E, Second>>>

typealias DerivativeGain<E, O> = Value<Fraction<O, Fraction<E, Second>>>

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

val <K : UnitKey> Value<K>.perMeterBySecond
  get() = Value<Fraction<K, Product<Meter, Second>>>(value)

val <K : UnitKey> Value<K>.perInchBySecond
  get() = perMeterBySecond / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFootBySecond
  get() = perMeterBySecond / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadianBySecond
  get() = Value<Fraction<K, Product<Radian, Second>>>(value)

val <K : UnitKey> Value<K>.perDegreeBySecond
  get() = perRadianBySecond / RADIANS_PER_DEGREES

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

val IntegralGain<Meter, Volt>.inVoltsPerMeterBySecond: Double
  get() = value

val IntegralGain<Meter, Volt>.inVoltsPerInchBySecond: Double
  get() = inVoltsPerMeterBySecond / METERS_PER_INCH

val IntegralGain<Meter, Volt>.inVoltsPerFootBySecond: Double
  get() = inVoltsPerMeterBySecond / METERS_PER_FOOT

val IntegralGain<Radian, Volt>.inVoltsPerRadianBySecond: Double
  get() = value

val IntegralGain<Radian, Volt>.inVoltsPerDegreeBySecond: Double
  get() = inVoltsPerRadianBySecond / RADIANS_PER_DEGREES

val IntegralGain<Radian, Volt>.inVoltsPerRotationsByMinute: Double
  get() = inVoltsPerRadianBySecond * SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

val DerivativeGain<Meter, Volt>.inVoltsPerMeterPerSecond: Double
  get() = value

val DerivativeGain<Meter, Volt>.inVoltsPerInchPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_INCH

val DerivativeGain<Meter, Volt>.inVoltsPerFootPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_FOOT

val DerivativeGain<Radian, Volt>.inVoltsPerRadianPerSecond: Double
  get() = value

val DerivativeGain<Radian, Volt>.inVoltsPerDegreePerSecond: Double
  get() = inVoltsPerRadianPerSecond / RADIANS_PER_DEGREES

val DerivativeGain<Radian, Volt>.inVoltsPerRotationsPerMinute: Double
  get() = inVoltsPerRadianPerSecond / SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

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

val IntegralGain<Meter, Ampere>.inAmpsPerMeterBySecond: Double
  get() = value

val IntegralGain<Meter, Ampere>.inAmpsPerInchBySecond: Double
  get() = inAmpsPerMeterBySecond / METERS_PER_INCH

val IntegralGain<Meter, Ampere>.inAmpsPerFootBySecond: Double
  get() = inAmpsPerMeterBySecond / METERS_PER_FOOT

val IntegralGain<Radian, Ampere>.inAmpsPerRadianBySecond: Double
  get() = value

val IntegralGain<Radian, Ampere>.inAmpsPerDegreeBySecond: Double
  get() = inAmpsPerRadianBySecond / RADIANS_PER_DEGREES

val IntegralGain<Radian, Ampere>.inAmpsPerRotationsByMinute: Double
  get() = inAmpsPerRadianBySecond * SECONDS_PER_MINUTE / RADIANS_PER_ROTATION

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
