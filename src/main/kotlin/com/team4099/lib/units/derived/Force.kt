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
import com.team4099.lib.units.inverse
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
  get() = value * 1.yotta.inverse

val Force.inZetanewtons: Double
  get() = value * 1.zeta.inverse

val Force.inExanewtons: Double
  get() = value * 1.exa.inverse

val Force.inPetanewtons: Double
  get() = value * 1.peta.inverse

val Force.inTeranewtons: Double
  get() = value * 1.tera.inverse

val Force.inGiganewtons: Double
  get() = value * 1.giga.inverse

val Force.inMeganewtons: Double
  get() = value * 1.mega.inverse

val Force.inKilonewtons: Double
  get() = value * 1.kilo.inverse

val Force.inHectonewtons: Double
  get() = value * 1.hecto.inverse

val Force.inDecanewtons: Double
  get() = value * 1.deca.inverse

val Force.inDecinewtons: Double
  get() = value * 1.deci.inverse

val Force.inCentinewtons: Double
  get() = value * 1.centi.inverse

val Force.inMillinewtons: Double
  get() = value * 1.milli.inverse

val Force.inMicronewtons: Double
  get() = value * 1.micro.inverse

val Force.inNanonewtons: Double
  get() = value * 1.nano.inverse

val Force.inPiconewtons: Double
  get() = value * 1.pico.inverse

val Force.inFemtonewtons: Double
  get() = value * 1.femto.inverse

val Force.inAttonewtons: Double
  get() = value * 1.atto.inverse

val Force.inZeptonewtons: Double
  get() = value * 1.zepto.inverse

val Force.inYoctonewtons: Double
  get() = value * 1.yocto.inverse
