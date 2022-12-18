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

typealias Energy = Value<Joule>

val Double.joules
  get() = Energy(this)

val Number.joules
  get() = Energy(this.toDouble())

val Energy.inJoules: Double
  get() = value

val Energy.inYottajoules: Double
  get() = value / 1.yotta

val Energy.inZetajoules: Double
  get() = value / 1.zeta

val Energy.inExajoules: Double
  get() = value / 1.exa

val Energy.inPetajoules: Double
  get() = value / 1.peta

val Energy.inTerajoules: Double
  get() = value / 1.tera

val Energy.inGigajoules: Double
  get() = value / 1.giga

val Energy.inMegajoules: Double
  get() = value / 1.mega

val Energy.inKilojoules: Double
  get() = value / 1.kilo

val Energy.inHectojoules: Double
  get() = value / 1.hecto

val Energy.inDecajoules: Double
  get() = value / 1.deca

val Energy.inDecijoules: Double
  get() = value / 1.deci

val Energy.inCentijoules: Double
  get() = value / 1.centi

val Energy.inMillijoules: Double
  get() = value / 1.milli

val Energy.inMicrojoules: Double
  get() = value / 1.micro

val Energy.inNanojoules: Double
  get() = value / 1.nano

val Energy.inPicojoules: Double
  get() = value / 1.pico

val Energy.inFemtojoules: Double
  get() = value / 1.femto

val Energy.inAttojoules: Double
  get() = value / 1.atto

val Energy.inZeptojoules: Double
  get() = value / 1.zepto

val Energy.inYoctojoules: Double
  get() = value / 1.yocto
