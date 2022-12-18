package com.team4099.lib.units

import kotlin.math.log
import kotlin.math.roundToInt

object Magnitude {
  const val YOTTA = 1E24
  const val ZETA = 1E21
  const val EXA = 1E18
  const val PETA = 1E15
  const val TERA = 1E12
  const val GIGA = 1E9
  const val MEGA = 1E6
  const val KILO = 1E3
  const val HECTO = 1E2
  const val DECA = 10
  const val DECI = 1E-1
  const val CENTI = 1E-2
  const val MILLI = 1E-3
  const val MICRO = 1E-6
  const val NANO = 1E-9
  const val PICO = 1E-12
  const val FEMTO = 1E-15
  const val ATTO = 1E-18
  const val ZEPTO = 1E-21
  const val YOCTO = 1E-24
}

val Double.yotta: Double
  get() = this * Magnitude.YOTTA

val Double.zeta: Double
  get() = this * Magnitude.ZETA

val Double.exa: Double
  get() = this * Magnitude.EXA

val Double.peta: Double
  get() = this * Magnitude.PETA

val Double.tera: Double
  get() = this * Magnitude.TERA

val Double.giga: Double
  get() = this * Magnitude.GIGA

val Double.mega: Double
  get() = this * Magnitude.MEGA

val Double.kilo: Double
  get() = this * Magnitude.KILO

val Double.hecto: Double
  get() = this * Magnitude.HECTO

val Double.deca: Double
  get() = this * Magnitude.DECA

val Double.deci: Double
  get() = this * Magnitude.DECI

val Double.centi: Double
  get() = this * Magnitude.CENTI

val Double.milli: Double
  get() = this * Magnitude.MILLI

val Double.micro: Double
  get() = this * Magnitude.MICRO

val Double.nano: Double
  get() = this * Magnitude.NANO

val Double.pico: Double
  get() = this * Magnitude.PICO

val Double.femto: Double
  get() = this * Magnitude.FEMTO

val Double.atto: Double
  get() = this * Magnitude.ATTO

val Double.zepto: Double
  get() = this * Magnitude.ZEPTO

val Double.yocto: Double
  get() = this * Magnitude.YOCTO

val Double.inverse: Double
  get() = "1E${-log(this, 10.0).roundToInt()}".toDouble()

val Number.yotta: Double
  get() = this.toDouble() * Magnitude.YOTTA

val Number.zeta: Double
  get() = this.toDouble() * Magnitude.ZETA

val Number.exa: Double
  get() = this.toDouble() * Magnitude.EXA

val Number.peta: Double
  get() = this.toDouble() * Magnitude.PETA

val Number.tera: Double
  get() = this.toDouble() * Magnitude.TERA

val Number.giga: Double
  get() = this.toDouble() * Magnitude.GIGA

val Number.mega: Double
  get() = this.toDouble() * Magnitude.MEGA

val Number.kilo: Double
  get() = this.toDouble() * Magnitude.KILO

val Number.hecto: Double
  get() = this.toDouble() * Magnitude.HECTO

val Number.deca: Double
  get() = this.toDouble() * Magnitude.DECA

val Number.deci: Double
  get() = this.toDouble() * Magnitude.DECI

val Number.centi: Double
  get() = this.toDouble() * Magnitude.CENTI

val Number.milli: Double
  get() = this.toDouble() * Magnitude.MILLI

val Number.micro: Double
  get() = this.toDouble() * Magnitude.MICRO

val Number.nano: Double
  get() = this.toDouble() * Magnitude.NANO

val Number.pico: Double
  get() = this.toDouble() * Magnitude.PICO

val Number.femto: Double
  get() = this.toDouble() * Magnitude.FEMTO

val Number.atto: Double
  get() = this.toDouble() * Magnitude.ATTO

val Number.zepto: Double
  get() = this.toDouble() * Magnitude.ZEPTO

val Number.yocto: Double
  get() = this.toDouble() * Magnitude.YOCTO

val Number.inverse: Double
  get() = "1E${-log(this.toDouble(), 10.0).roundToInt()}".toDouble()
