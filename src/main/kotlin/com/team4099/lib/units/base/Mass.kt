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
  get() = (value * Magnitude.KILO) / 1.yotta

val Mass.inZetagrams: Double
  get() = (value * Magnitude.KILO) / 1.zeta

val Mass.inExagrams: Double
  get() = (value * Magnitude.KILO) / 1.exa

val Mass.inPetagrams: Double
  get() = (value * Magnitude.KILO) / 1.peta

val Mass.inTeragrams: Double
  get() = (value * Magnitude.KILO) / 1.tera

val Mass.inGigagrams: Double
  get() = (value * Magnitude.KILO) / 1.giga

val Mass.inMegagrams: Double
  get() = (value * Magnitude.KILO) / 1.mega

val Mass.inKilograms: Double
  get() = (value * Magnitude.KILO) / 1.kilo

val Mass.inHectograms: Double
  get() = (value * Magnitude.KILO) / 1.hecto

val Mass.inDecagrams: Double
  get() = (value * Magnitude.KILO) * 1.deca

val Mass.inDecigrams: Double
  get() = (value * Magnitude.KILO) * 1.deci

val Mass.inCentigrams: Double
  get() = (value * Magnitude.KILO) * 1.centi

val Mass.inMilligrams: Double
  get() = (value * Magnitude.KILO) * 1.milli

val Mass.inMicrograms: Double
  get() = (value * Magnitude.KILO) * 1.micro

val Mass.inNanograms: Double
  get() = (value * Magnitude.KILO) * 1.nano

val Mass.inPicograms: Double
  get() = (value * Magnitude.KILO) * 1.pico

val Mass.inFemtograms: Double
  get() = (value * Magnitude.KILO) * 1.femto

val Mass.inAttograms: Double
  get() = (value * Magnitude.KILO) * 1.atto

val Mass.inZeptograms: Double
  get() = (value * Magnitude.KILO) * 1.zepto

val Mass.inYoctograms: Double
  get() = (value * Magnitude.KILO) * 1.yocto
