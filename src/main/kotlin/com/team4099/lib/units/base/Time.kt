package com.team4099.lib.units.base

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.attoinverse
import com.team4099.lib.units.centiinverse
import com.team4099.lib.units.deciinverse
import com.team4099.lib.units.femtoinverse
import com.team4099.lib.units.microinverse
import com.team4099.lib.units.milliinverse
import com.team4099.lib.units.nanoinverse
import com.team4099.lib.units.picoinverse
import com.team4099.lib.units.yoctoinverse
import com.team4099.lib.units.zeptoinverse

object Second : UnitKey

typealias Time = Value<Second>

internal const val SECONDS_PER_MINUTE = 60

internal const val SECONDS_PER_HOUR = SECONDS_PER_MINUTE * 60

val Double.seconds: Time
  get() = Time(this)

val Double.minutes: Time
  get() = Time(this * SECONDS_PER_MINUTE)

val Double.hours: Time
  get() = Time(this * SECONDS_PER_HOUR)

val Number.seconds: Time
  get() = toDouble().seconds

val Number.minutes: Time
  get() = toDouble().minutes

val Number.hours: Time
  get() = toDouble().hours

val Time.inSeconds: Double
  get() = value

val Time.inMinutes: Double
  get() = value / SECONDS_PER_MINUTE

val Time.inHours: Double
  get() = value / SECONDS_PER_HOUR

val Time.inDeciseconds: Double
  get() = value * 1.deciinverse

val Time.inCentiseconds: Double
  get() = value * 1.centiinverse

val Time.inMilliseconds: Double
  get() = value * 1.milliinverse

val Time.inMicroseconds: Double
  get() = value * 1.microinverse

val Time.inNanoseconds: Double
  get() = value * 1.nanoinverse

val Time.inPicoseconds: Double
  get() = value * 1.picoinverse

val Time.inFemtoseconds: Double
  get() = value * 1.femtoinverse

val Time.inAttoseconds: Double
  get() = value * 1.attoinverse

val Time.inZeptoseconds: Double
  get() = value * 1.zeptoinverse

val Time.inYoctoseconds: Double
  get() = value * 1.yoctoinverse
