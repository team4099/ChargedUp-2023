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

typealias Newton = Fraction<Product<Kilogram, Meter>, Squared<Second>>

typealias Force = Value<Newton>

val Double.newtons
  get() = Force(this)

val Number.newtons
  get() = Force(this.toDouble())

val Force.inNewtons: Double
  get() = value

val Force.inYottanewtons: Double
  get() = value * 1.yottainverse

val Force.inZetanewtons: Double
  get() = value * 1.zetainverse

val Force.inExanewtons: Double
  get() = value * 1.exainverse

val Force.inPetanewtons: Double
  get() = value * 1.petainverse

val Force.inTeranewtons: Double
  get() = value * 1.terainverse

val Force.inGiganewtons: Double
  get() = value * 1.gigainverse

val Force.inMeganewtons: Double
  get() = value * 1.megainverse

val Force.inKilonewtons: Double
  get() = value * 1.kiloinverse

val Force.inHectonewtons: Double
  get() = value * 1.hectoinverse

val Force.inDecanewtons: Double
  get() = value * 1.decainverse

val Force.inDecinewtons: Double
  get() = value * 1.deciinverse

val Force.inCentinewtons: Double
  get() = value * 1.centiinverse

val Force.inMillinewtons: Double
  get() = value * 1.milliinverse

val Force.inMicronewtons: Double
  get() = value * 1.microinverse

val Force.inNanonewtons: Double
  get() = value * 1.nanoinverse

val Force.inPiconewtons: Double
  get() = value * 1.picoinverse

val Force.inFemtonewtons: Double
  get() = value * 1.femtoinverse

val Force.inAttonewtons: Double
  get() = value * 1.attoinverse

val Force.inZeptonewtons: Double
  get() = value * 1.zeptoinverse

val Force.inYoctonewtons: Double
  get() = value * 1.yoctoinverse
