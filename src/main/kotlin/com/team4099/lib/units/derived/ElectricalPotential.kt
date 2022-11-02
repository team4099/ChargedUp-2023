package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Value

val BATTERY_VOLTAGE = 12.volts

typealias Volt = Fraction<Joule, Coulomb>

typealias ElectricalPotential = Value<Volt>

val Double.volts
  get() = ElectricalPotential(this)

val Number.volts
  get() = toDouble().volts

val ElectricalPotential.inVolts
  get() = value

val ElectricalPotential.inPercentOutput
  get() = this / BATTERY_VOLTAGE
