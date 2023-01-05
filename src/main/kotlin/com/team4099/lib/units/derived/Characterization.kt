package com.team4099.lib.units.derived

import com.team4099.lib.units.Acceleration
import com.team4099.lib.units.Fraction
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.Velocity
import com.team4099.lib.units.base.Meter

typealias StaticFeedforward = Value<Volt>

typealias VelocityFeedforward<T> = Value<Fraction<Volt, Velocity<T>>>

typealias AccelerationFeedforward<T> = Value<Fraction<Volt, Acceleration<T>>>

typealias AngularGravityFeedforward = Value<Fraction<Volt, Radian>>

typealias LinearGravityFeedforward = Value<Volt>

val <K : UnitKey> Value<K>.perMeterPerSecondPerSecond
  get() = Value<Fraction<K, Acceleration<Meter>>>(value)

val <K : UnitKey> Value<K>.perRadianPerSecondPerSecond
  get() = Value<Fraction<K, Acceleration<Radian>>>(value)

val AccelerationFeedforward<Radian>.inVoltsPerRadianPerSecondPerSecond
  get() = value

val AccelerationFeedforward<Meter>.inVoltsPerMeterPerSecondPerSecond
  get() = value
