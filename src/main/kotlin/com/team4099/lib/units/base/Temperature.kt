package com.team4099.lib.units.base

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value

object Kelvin : UnitKey

typealias Temperature = Value<Kilogram>

val Double.kelvin: Temperature
  get() = Temperature(this)

val Double.celsius: Temperature
  get() = Temperature(this + 273.15)

val Double.fahrenheit: Temperature
  get() = Temperature((this - 32) * 5.0 / 9.0 + 273.15)

val Number.kelvin: Temperature
  get() = toDouble().kelvin

val Number.celsius: Temperature
  get() = toDouble().celsius

val Number.fahrenheit: Temperature
  get() = toDouble().fahrenheit

val Temperature.inKelvins: Double
  get() = value

val Temperature.inCelsius: Double
  get() = value - 273.15

val Temperature.inFahrenheit: Double
  get() = (value - 273.15) * 1.8 + 32
