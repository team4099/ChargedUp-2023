package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.Squared
import com.team4099.lib.units.Value
import com.team4099.lib.units.atto
import com.team4099.lib.units.base.Kilogram
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.Second
import com.team4099.lib.units.centi
import com.team4099.lib.units.deca
import com.team4099.lib.units.deci
import com.team4099.lib.units.exa
import com.team4099.lib.units.femto
import com.team4099.lib.units.giga
import com.team4099.lib.units.hecto
import com.team4099.lib.units.kilo
import com.team4099.lib.units.mega
import com.team4099.lib.units.micro
import com.team4099.lib.units.milli
import com.team4099.lib.units.nano
import com.team4099.lib.units.peta
import com.team4099.lib.units.pico
import com.team4099.lib.units.tera
import com.team4099.lib.units.yocto
import com.team4099.lib.units.yotta
import com.team4099.lib.units.zepto
import com.team4099.lib.units.zeta

typealias Newton = Fraction<Product<Kilogram, Meter>, Squared<Second>>

typealias Force = Value<Newton>

val Double.newtons
  get() = Force(this)

val Number.newtons
  get() = Force(this.toDouble())

val Force.inNewtons: Double
  get() = value

val Force.inYottanewtons: Double
  get() = value / 1.yotta

val Force.inZetanewtons: Double
  get() = value / 1.zeta

val Force.inExanewtons: Double
  get() = value / 1.exa

val Force.inPetanewtons: Double
  get() = value / 1.peta

val Force.inTeranewtons: Double
  get() = value / 1.tera

val Force.inGiganewtons: Double
  get() = value / 1.giga

val Force.inMeganewtons: Double
  get() = value / 1.mega

val Force.inKilonewtons: Double
  get() = value / 1.kilo

val Force.inHectonewtons: Double
  get() = value / 1.hecto

val Force.inDecanewtons: Double
  get() = value / 1.deca

val Force.inDecinewtons: Double
  get() = value / 1.deci

val Force.inCentinewtons: Double
  get() = value / 1.centi

val Force.inMillinewtons: Double
  get() = value / 1.milli

val Force.inMicronewtons: Double
  get() = value / 1.micro

val Force.inNanonewtons: Double
  get() = value / 1.nano

val Force.inPiconewtons: Double
  get() = value / 1.pico

val Force.inFemtonewtons: Double
  get() = value / 1.femto

val Force.inAttonewtons: Double
  get() = value / 1.atto

val Force.inZeptonewtons: Double
  get() = value / 1.zepto

val Force.inYoctonewtons: Double
  get() = value / 1.yocto
