package com.team4099

import com.team4099.lib.units.base.SECONDS_PER_HOUR
import com.team4099.lib.units.base.SECONDS_PER_MINUTE
import com.team4099.lib.units.base.hours
import com.team4099.lib.units.base.inCentiseconds
import com.team4099.lib.units.base.inDeciseconds
import com.team4099.lib.units.base.inHours
import com.team4099.lib.units.base.inMilliseconds
import com.team4099.lib.units.base.inMinutes
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.minutes
import com.team4099.lib.units.base.seconds
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals

class TimeTest {
  private val kEpsilon = 1E-9
  private val kDelta = 1E-3
  private val kSigma = 1E6
  private val kAlpha = 1E9
  private val kBeta = 1E12

  @Test
  fun testSecondsToMinutes() {
    val seconds = 4099.seconds
    assertEquals(seconds.inMinutes, seconds.inSeconds / SECONDS_PER_MINUTE, kEpsilon)
  }

  @Test
  fun testSecondsToHours() {
    val seconds = 4099.seconds
    assertEquals(seconds.inHours, seconds.inSeconds / SECONDS_PER_HOUR, kEpsilon)
  }

  @Test
  fun testMinutesToSeconds() {
    val minutes = 4099.minutes
    assertEquals(minutes.inSeconds, minutes.inSeconds * SECONDS_PER_MINUTE, kEpsilon)
  }

  @Test
  fun testHoursToSeconds() {
    val hours = 4099.hours
    assertEquals(hours.inSeconds, hours.inHours * SECONDS_PER_HOUR, kEpsilon)
  }

  @Test
  fun testSecondsToDeciseconds() {
    val seconds = 4099.seconds
    assertEquals(seconds.inDeciseconds, 4099E1, kEpsilon)
  }

  @Test
  fun testSecondsToCentiseconds() {
    val seconds = 4099.seconds
    assertEquals(seconds.inCentiseconds, 4099E2, kEpsilon)
  }

  @Test
  fun testSecondsToMilliseconds() {
    val seconds = 4099.seconds
    assertEquals(seconds.inMilliseconds, 4099E3, kEpsilon)
  }
}
