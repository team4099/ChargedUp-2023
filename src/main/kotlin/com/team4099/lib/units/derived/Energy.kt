package com.team4099.lib.units.derived

import com.team4099.lib.units.Fraction
import com.team4099.lib.units.Product
import com.team4099.lib.units.Squared
import com.team4099.lib.units.Value
import com.team4099.lib.units.attoinverse
import com.team4099.lib.units.base.Kilogram
import com.team4099.lib.units.base.Meter
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

typealias Joule = Fraction<Product<Kilogram, Squared<Meter>>, Squared<Second>>

typealias Energy = Value<Joule>

val Double.joules
  get() = Energy(this)

val Number.joules
  get() = Energy(this.toDouble())

val Energy.inJoules: Double
  get() = value

val Energy.inYottajoules: Double
  get() = value * 1.yottainverse

val Energy.inZetajoules: Double
  get() = value * 1.zetainverse

val Energy.inExajoules: Double
  get() = value * 1.exainverse

val Energy.inPetajoules: Double
  get() = value * 1.petainverse

val Energy.inTerajoules: Double
  get() = value * 1.terainverse

val Energy.inGigajoules: Double
  get() = value * 1.gigainverse

val Energy.inMegajoules: Double
  get() = value * 1.megainverse

val Energy.inKilojoules: Double
  get() = value * 1.kiloinverse

val Energy.inHectojoules: Double
  get() = value * 1.hectoinverse

val Energy.inDecajoules: Double
  get() = value * 1.decainverse

val Energy.inDecijoules: Double
  get() = value * 1.deciinverse

val Energy.inCentijoules: Double
  get() = value * 1.centiinverse

val Energy.inMillijoules: Double
  get() = value * 1.milliinverse

val Energy.inMicrojoules: Double
  get() = value * 1.microinverse

val Energy.inNanojoules: Double
  get() = value * 1.nanoinverse

val Energy.inPicojoules: Double
  get() = value * 1.picoinverse

val Energy.inFemtojoules: Double
  get() = value * 1.femtoinverse

val Energy.inAttojoules: Double
  get() = value * 1.attoinverse

val Energy.inZeptojoules: Double
  get() = value * 1.zeptoinverse

val Energy.inYoctojoules: Double
  get() = value * 1.yoctoinverse
