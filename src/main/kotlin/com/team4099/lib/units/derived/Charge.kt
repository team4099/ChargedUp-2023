package com.team4099.lib.units.derived

import com.team4099.lib.units.Product
import com.team4099.lib.units.Value
import com.team4099.lib.units.atto
import com.team4099.lib.units.base.Ampere
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

typealias Coulomb = Product<Ampere, Second>

typealias Charge = Value<Coulomb>

val Double.coulombs
  get() = Charge(this)

val Number.coulombs
  get() = Charge(this.toDouble())

val Charge.inCoulombs: Double
  get() = value

val Charge.inYottacoulombs: Double
  get() = value / 1.yotta

val Charge.inZetacoulombs: Double
  get() = value / 1.zeta

val Charge.inExacoulombs: Double
  get() = value / 1.exa

val Charge.inPetacoulombs: Double
  get() = value / 1.peta

val Charge.inTeracoulombs: Double
  get() = value / 1.tera

val Charge.inGigacoulombs: Double
  get() = value / 1.giga

val Charge.inMegacoulombs: Double
  get() = value / 1.mega

val Charge.inKilocoulombs: Double
  get() = value / 1.kilo

val Charge.inHectocoulombs: Double
  get() = value / 1.hecto

val Charge.inDecacoulombs: Double
  get() = value / 1.deca

val Charge.inDecicoulombs: Double
  get() = value / 1.deci

val Charge.inCenticoulombs: Double
  get() = value / 1.centi

val Charge.inMillicoulombs: Double
  get() = value / 1.milli

val Charge.inMicrocoulombs: Double
  get() = value / 1.micro

val Charge.inNanocoulombs: Double
  get() = value / 1.nano

val Charge.inPicocoulombs: Double
  get() = value / 1.pico

val Charge.inFemtocoulombs: Double
  get() = value / 1.femto

val Charge.inAttocoulombs: Double
  get() = value / 1.atto

val Charge.inZeptocoulombs: Double
  get() = value / 1.zepto

val Charge.inYoctocoulombs: Double
  get() = value / 1.yocto
