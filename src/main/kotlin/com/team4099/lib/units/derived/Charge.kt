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

typealias Coulomb = Product<Ampere, Second>

typealias Charge = Value<Coulomb>

val Double.coulombs
  get() = Charge(this)

val Number.coulombs
  get() = Charge(this.toDouble())

val Charge.inCoulombs: Double
  get() = value

val Charge.inYottacoulombs: Double
  get() = value * 1.yotta.inverse

val Charge.inZetacoulombs: Double
  get() = value * 1.zeta.inverse

val Charge.inExacoulombs: Double
  get() = value * 1.exa.inverse

val Charge.inPetacoulombs: Double
  get() = value * 1.peta.inverse

val Charge.inTeracoulombs: Double
  get() = value * 1.tera.inverse

val Charge.inGigacoulombs: Double
  get() = value * 1.giga.inverse

val Charge.inMegacoulombs: Double
  get() = value * 1.mega.inverse

val Charge.inKilocoulombs: Double
  get() = value * 1.kilo.inverse

val Charge.inHectocoulombs: Double
  get() = value * 1.hecto.inverse

val Charge.inDecacoulombs: Double
  get() = value * 1.deca.inverse

val Charge.inDecicoulombs: Double
  get() = value * 1.deci.inverse

val Charge.inCenticoulombs: Double
  get() = value * 1.centi.inverse

val Charge.inMillicoulombs: Double
  get() = value * 1.milli.inverse

val Charge.inMicrocoulombs: Double
  get() = value * 1.micro.inverse

val Charge.inNanocoulombs: Double
  get() = value * 1.nano.inverse

val Charge.inPicocoulombs: Double
  get() = value * 1.pico.inverse

val Charge.inFemtocoulombs: Double
  get() = value * 1.femto.inverse

val Charge.inAttocoulombs: Double
  get() = value * 1.atto.inverse

val Charge.inZeptocoulombs: Double
  get() = value * 1.zepto.inverse

val Charge.inYoctocoulombs: Double
  get() = value * 1.yocto.inverse
