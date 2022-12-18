package com.team4099.lib.units.base

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.atto
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

object Ampere : UnitKey

typealias Current = Value<Ampere>

val Double.amps: Current
  get() = Current(this)

val Number.amps: Current
  get() = this.toDouble().amps

val Current.inAmperes
  get() = value

val Current.inYottaamps
  get() = value * 1.yotta.inverse

val Current.inZetaamps
  get() = value * 1.zeta.inverse

val Current.inExaamps
  get() = value * 1.exa.inverse

val Current.inPetaamps
  get() = value * 1.peta.inverse

val Current.inTeraamps
  get() = value * 1.tera.inverse

val Current.inGigaamps
  get() = value * 1.giga.inverse

val Current.inMegaamps
  get() = value * 1.mega.inverse

val Current.inKiloamps
  get() = value * 1.kilo.inverse

val Current.inHectoamps
  get() = value * 1.hecto.inverse

val Current.inDecaamps
  get() = value * 1.deca.inverse

val Current.inDeciamps
  get() = value * 1.deci.inverse

val Current.inCentiamps
  get() = value * 1.centi.inverse

val Current.inMilliamps
  get() = value * 1.milli.inverse

val Current.inMicroamps
  get() = value * 1.micro.inverse

val Current.inNanoamps
  get() = value * 1.nano.inverse

val Current.inPicoamps
  get() = value * 1.pico.inverse

val Current.inFemtoamps
  get() = value * 1.femto.inverse

val Current.inAttoamps
  get() = value * 1.atto.inverse

val Current.inZeptoamps
  get() = value * 1.zepto.inverse

val Current.inYoctoamps
  get() = value * 1.yocto.inverse
