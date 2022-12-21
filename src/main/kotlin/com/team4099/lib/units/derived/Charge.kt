package com.team4099.lib.units.derived

import com.team4099.lib.units.Product
import com.team4099.lib.units.Value
import com.team4099.lib.units.attoinverse
import com.team4099.lib.units.base.Ampere
import com.team4099.lib.units.base.Second
import com.team4099.lib.units.centiinverse
import com.team4099.lib.units.decainverse
import com.team4099.lib.units.deciinverse
import com.team4099.lib.units.exainverse
import com.team4099.lib.units.femtoinverse
import com.team4099.lib.units.gigainverse
import com.team4099.lib.units.hectoinverse
import com.team4099.lib.units.kiloinverse
import com.team4099.lib.units.megainverse
import com.team4099.lib.units.microinverse
import com.team4099.lib.units.milliinverse
import com.team4099.lib.units.nanoinverse
import com.team4099.lib.units.petainverse
import com.team4099.lib.units.picoinverse
import com.team4099.lib.units.terainverse
import com.team4099.lib.units.yoctoinverse
import com.team4099.lib.units.yottainverse
import com.team4099.lib.units.zeptoinverse
import com.team4099.lib.units.zetainverse

typealias Coulomb = Product<Ampere, Second>

typealias Charge = Value<Coulomb>

val Double.coulombs
  get() = Charge(this)

val Number.coulombs
  get() = Charge(this.toDouble())

val Charge.inCoulombs: Double
  get() = value

val Charge.inYottacoulombs: Double
  get() = value * 1.yottainverse

val Charge.inZetacoulombs: Double
  get() = value * 1.zetainverse

val Charge.inExacoulombs: Double
  get() = value * 1.exainverse

val Charge.inPetacoulombs: Double
  get() = value * 1.petainverse

val Charge.inTeracoulombs: Double
  get() = value * 1.terainverse

val Charge.inGigacoulombs: Double
  get() = value * 1.gigainverse

val Charge.inMegacoulombs: Double
  get() = value * 1.megainverse

val Charge.inKilocoulombs: Double
  get() = value * 1.kiloinverse

val Charge.inHectocoulombs: Double
  get() = value * 1.hectoinverse

val Charge.inDecacoulombs: Double
  get() = value * 1.decainverse

val Charge.inDecicoulombs: Double
  get() = value * 1.deciinverse

val Charge.inCenticoulombs: Double
  get() = value * 1.centiinverse

val Charge.inMillicoulombs: Double
  get() = value * 1.milliinverse

val Charge.inMicrocoulombs: Double
  get() = value * 1.microinverse

val Charge.inNanocoulombs: Double
  get() = value * 1.nanoinverse

val Charge.inPicocoulombs: Double
  get() = value * 1.picoinverse

val Charge.inFemtocoulombs: Double
  get() = value * 1.femtoinverse

val Charge.inAttocoulombs: Double
  get() = value * 1.attoinverse

val Charge.inZeptocoulombs: Double
  get() = value * 1.zeptoinverse

val Charge.inYoctocoulombs: Double
  get() = value * 1.yoctoinverse
