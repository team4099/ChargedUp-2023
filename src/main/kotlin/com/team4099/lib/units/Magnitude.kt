package com.team4099.lib.units

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

  const val YOTTA_INVERSE = 1E-24
  const val ZETA_INVERSE = 1E-21
  const val EXA_INVERSE = 1E-18
  const val PETA_INVERSE = 1E-15
  const val TERA_INVERSE = 1E-12
  const val GIGA_INVERSE = 1E-9
  const val MEGA_INVERSE = 1E-6
  const val KILO_INVERSE = 1E-3
  const val HECTO_INVERSE = 1E-2
  const val DECA_INVERSE = 1E-1
  const val DECI_INVERSE = 1E1
  const val CENTI_INVERSE = 1E2
  const val MILLI_INVERSE = 1E3
  const val MICRO_INVERSE = 1E6
  const val NANO_INVERSE = 1E9
  const val PICO_INVERSE = 1E12
  const val FEMTO_INVERSE = 1E15
  const val ATTO_INVERSE = 1E18
  const val ZEPTO_INVERSE = 1E21
  const val YOCTO_INVERSE = 1E24
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

val Double.yottainverse: Double
  get() = this * Magnitude.YOTTA_INVERSE

val Double.zetainverse: Double
  get() = this * Magnitude.ZETA_INVERSE

val Double.exainverse: Double
  get() = this * Magnitude.EXA_INVERSE

val Double.petainverse: Double
  get() = this * Magnitude.PETA_INVERSE

val Double.terainverse: Double
  get() = this * Magnitude.TERA_INVERSE

val Double.gigainverse: Double
  get() = this * Magnitude.GIGA_INVERSE

val Double.megainverse: Double
  get() = this * Magnitude.MEGA_INVERSE

val Double.kiloinverse: Double
  get() = this * Magnitude.KILO_INVERSE

val Double.hectoinverse: Double
  get() = this * Magnitude.HECTO_INVERSE

val Double.decainverse: Double
  get() = this * Magnitude.DECA_INVERSE

val Double.deciinverse: Double
  get() = this * Magnitude.DECI_INVERSE

val Double.centiinverse: Double
  get() = this * Magnitude.CENTI_INVERSE

val Double.milliinverse: Double
  get() = this * Magnitude.MILLI_INVERSE

val Double.microinverse: Double
  get() = this * Magnitude.MICRO_INVERSE

val Double.nanoinverse: Double
  get() = this * Magnitude.NANO_INVERSE

val Double.picoinverse: Double
  get() = this * Magnitude.PICO_INVERSE

val Double.femtoinverse: Double
  get() = this * Magnitude.FEMTO_INVERSE

val Double.attoinverse: Double
  get() = this * Magnitude.ATTO_INVERSE

val Double.zeptoinverse: Double
  get() = this * Magnitude.ZEPTO_INVERSE

val Double.yoctoinverse: Double
  get() = this * Magnitude.YOCTO_INVERSE

val Number.yottainverse: Double
  get() = this.toDouble() * Magnitude.YOTTA_INVERSE

val Number.zetainverse: Double
  get() = this.toDouble() * Magnitude.ZETA_INVERSE

val Number.exainverse: Double
  get() = this.toDouble() * Magnitude.EXA_INVERSE

val Number.petainverse: Double
  get() = this.toDouble() * Magnitude.PETA_INVERSE

val Number.terainverse: Double
  get() = this.toDouble() * Magnitude.TERA_INVERSE

val Number.gigainverse: Double
  get() = this.toDouble() * Magnitude.GIGA_INVERSE

val Number.megainverse: Double
  get() = this.toDouble() * Magnitude.MEGA_INVERSE

val Number.kiloinverse: Double
  get() = this.toDouble() * Magnitude.KILO_INVERSE

val Number.hectoinverse: Double
  get() = this.toDouble() * Magnitude.HECTO_INVERSE

val Number.decainverse: Double
  get() = this.toDouble() * Magnitude.DECA_INVERSE

val Number.deciinverse: Double
  get() = this.toDouble() * Magnitude.DECI_INVERSE

val Number.centiinverse: Double
  get() = this.toDouble() * Magnitude.CENTI_INVERSE

val Number.milliinverse: Double
  get() = this.toDouble() * Magnitude.MILLI_INVERSE

val Number.microinverse: Double
  get() = this.toDouble() * Magnitude.MICRO_INVERSE

val Number.nanoinverse: Double
  get() = this.toDouble() * Magnitude.NANO_INVERSE

val Number.picoinverse: Double
  get() = this.toDouble() * Magnitude.PICO_INVERSE

val Number.femtoinverse: Double
  get() = this.toDouble() * Magnitude.FEMTO_INVERSE

val Number.attoinverse: Double
  get() = this.toDouble() * Magnitude.ATTO_INVERSE

val Number.zeptoinverse: Double
  get() = this.toDouble() * Magnitude.ZEPTO_INVERSE

val Number.yoctoinverse: Double
  get() = this.toDouble() * Magnitude.YOCTO_INVERSE
