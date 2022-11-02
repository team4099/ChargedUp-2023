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

typealias Joule = Fraction<Product<Kilogram, Squared<Meter>>, Squared<Second>>

typealias Energy = Value<Newton>

val Double.joules
  get() = Energy(this)

val Number.joules
  get() = Energy(this.toDouble())

val Force.inJoules: Double
  get() = value

val Force.inYottajoules: Double
  get() = value * 1.yotta

val Force.inZetajoules: Double
  get() = value * 1.zeta

val Force.inExajoules: Double
  get() = value * 1.exa

val Force.inPetajoules: Double
  get() = value * 1.peta

val Force.inTerajoules: Double
  get() = value * 1.tera

val Force.inGigajoules: Double
  get() = value * 1.giga

val Force.inMegajoules: Double
  get() = value * 1.mega

val Force.inKilojoules: Double
  get() = value * 1.kilo

val Force.inHectojoules: Double
  get() = value * 1.hecto

val Force.inDecajoules: Double
  get() = value * 1.deca

val Force.inDecijoules: Double
  get() = value * 1.deci

val Force.inCentijoules: Double
  get() = value * 1.centi

val Force.inMillijoules: Double
  get() = value * 1.milli

val Force.inMicrojoules: Double
  get() = value * 1.micro

val Force.inNanojoules: Double
  get() = value * 1.nano

val Force.inPicojoules: Double
  get() = value * 1.pico

val Force.inFemtojoules: Double
  get() = value * 1.femto

val Force.inAttojoules: Double
  get() = value * 1.atto

val Force.inZeptojoules: Double
  get() = value * 1.zepto

val Force.inYoctojoules: Double
  get() = value * 1.yocto
