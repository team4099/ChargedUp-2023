package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.base.METERS_PER_FOOT
import com.team4099.lib.units.base.METERS_PER_INCH
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.Second

typealias ProportionalGain<T, K> = Value<Fraction<T, K>>

typealias IntegralGain<T, K> = Value<Fraction<Product<T, Second>, K>>

typealias DerivativeGain<T, K> = Value<Fraction<Product<T, Fraction<T, Second>>, K>>

val <K : UnitKey> Value<K>.perMeter
  get() = ProportionalGain<K, Meter>(value)

val <K : UnitKey> Value<K>.perInch
  get() = perMeter / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFoot
  get() = perMeter / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadian
  get() = ProportionalGain<K, Radian>(value)

val <K : UnitKey> Value<K>.perDegree
  get() = perRadian / RADIANS_PER_DEGREES

val <K : UnitKey> Value<K>.perMeterBySecond
  get() = IntegralGain<K, Meter>(value)

val <K : UnitKey> Value<K>.perInchBySecond
  get() = perMeterBySecond / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFootBySecond
  get() = perMeterBySecond / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadianBySecond
  get() = IntegralGain<K, Radian>(value)

val <K : UnitKey> Value<K>.perDegreeBySecond
  get() = perRadianBySecond / RADIANS_PER_DEGREES

val <K : UnitKey> Value<K>.perMeterPerSecond
  get() = DerivativeGain<K, Meter>(value)

val <K : UnitKey> Value<K>.perInchPerSecond
  get() = perMeterPerSecond / METERS_PER_INCH

val <K : UnitKey> Value<K>.perFootPerSecond
  get() = perMeterPerSecond / METERS_PER_FOOT

val <K : UnitKey> Value<K>.perRadianPerSecond
  get() = DerivativeGain<K, Radian>(value)

val <K : UnitKey> Value<K>.perDegreePerSecond
  get() = perRadianPerSecond / RADIANS_PER_DEGREES

val ProportionalGain<Volt, Meter>.inVoltsPerMeter: Double
  get() = value

val ProportionalGain<Volt, Meter>.inVoltsPerInch: Double
  get() = inVoltsPerMeter / METERS_PER_INCH

val ProportionalGain<Volt, Meter>.inVoltsPerFoot: Double
  get() = inVoltsPerMeter / METERS_PER_FOOT

val ProportionalGain<Volt, Radian>.inVoltsPerRadian: Double
  get() = value

val ProportionalGain<Volt, Radian>.inVoltsPerDegrees: Double
  get() = inVoltsPerRadian / RADIANS_PER_DEGREES

val IntegralGain<Volt, Meter>.inVoltsPerMeterBySecond: Double
  get() = value

val IntegralGain<Volt, Meter>.inVoltsPerInchBySecond: Double
  get() = inVoltsPerMeterBySecond / METERS_PER_INCH

val IntegralGain<Volt, Meter>.inVoltsPerFootBySecond: Double
  get() = inVoltsPerMeterBySecond / METERS_PER_FOOT

val IntegralGain<Volt, Radian>.inVoltsPerRadianBySecond: Double
  get() = value

val IntegralGain<Volt, Radian>.inVoltsPerDegreeBySecond: Double
  get() = inVoltsPerRadianBySecond / RADIANS_PER_DEGREES

val DerivativeGain<Volt, Meter>.inVoltsPerMeterPerSecond: Double
  get() = value

val DerivativeGain<Volt, Meter>.inVoltsPerInchPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_INCH

val DerivativeGain<Volt, Meter>.inVoltsPerFootPerSecond: Double
  get() = inVoltsPerMeterPerSecond / METERS_PER_FOOT

val DerivativeGain<Volt, Radian>.inVoltsPerRadianPerSecond: Double
  get() = value

val DerivativeGain<Volt, Radian>.inVoltsPerDegreePerSecond: Double
  get() = inVoltsPerRadianPerSecond / RADIANS_PER_DEGREES
