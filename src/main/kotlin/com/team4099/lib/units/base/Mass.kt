package com.team4099.lib.units.base

import com.team4099.lib.units.Magnitude
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

object Kilogram : UnitKey

typealias Mass = Value<Kilogram>

internal const val GRAMS_PER_POUND = 453.5924

val Double.grams: Mass
  get() = Mass(this / 1000)

val Double.pounds: Mass
  get() = Mass(this * GRAMS_PER_POUND)

val Number.grams: Mass
  get() = toDouble().grams

val Number.pounds: Mass
  get() = toDouble().pounds

val Mass.inGrams: Double
  get() = value * Magnitude.KILO

val Mass.inPounds: Double
  get() = (value * Magnitude.KILO) / GRAMS_PER_POUND

val Mass.inYottagrams: Double
  get() = (value * Magnitude.KILO) * 1.yotta.inverse

val Mass.inZetagrams: Double
  get() = (value * Magnitude.KILO) * 1.zeta.inverse

val Mass.inExagrams: Double
  get() = (value * Magnitude.KILO) * 1.exa.inverse

val Mass.inPetagrams: Double
  get() = (value * Magnitude.KILO) * 1.peta.inverse

val Mass.inTeragrams: Double
  get() = (value * Magnitude.KILO) * 1.tera.inverse

val Mass.inGigagrams: Double
  get() = (value * Magnitude.KILO) * 1.giga.inverse

val Mass.inMegagrams: Double
  get() = (value * Magnitude.KILO) * 1.mega.inverse

val Mass.inKilograms: Double
  get() = (value * Magnitude.KILO) * 1.kilo.inverse

val Mass.inHectograms: Double
  get() = (value * Magnitude.KILO) * 1.hecto.inverse

val Mass.inDecagrams: Double
  get() = (value * Magnitude.KILO) * 1.deca.inverse

val Mass.inDecigrams: Double
  get() = (value * Magnitude.KILO) * 1.deci.inverse

val Mass.inCentigrams: Double
  get() = (value * Magnitude.KILO) * 1.centi.inverse

val Mass.inMilligrams: Double
  get() = (value * Magnitude.KILO) * 1.milli.inverse

val Mass.inMicrograms: Double
  get() = (value * Magnitude.KILO) * 1.micro.inverse

val Mass.inNanograms: Double
  get() = (value * Magnitude.KILO) * 1.nano.inverse

val Mass.inPicograms: Double
  get() = (value * Magnitude.KILO) * 1.pico.inverse

val Mass.inFemtograms: Double
  get() = (value * Magnitude.KILO) * 1.femto.inverse

val Mass.inAttograms: Double
  get() = (value * Magnitude.KILO) * 1.atto.inverse

val Mass.inZeptograms: Double
  get() = (value * Magnitude.KILO) * 1.zepto.inverse

val Mass.inYoctograms: Double
  get() = (value * Magnitude.KILO) * 1.yocto.inverse
