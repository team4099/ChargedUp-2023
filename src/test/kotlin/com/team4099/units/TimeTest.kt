package com.team4099.units

import com.team4099.lib.units.base.SECONDS_PER_HOUR
import com.team4099.lib.units.base.SECONDS_PER_MINUTE
import com.team4099.lib.units.base.hours
import com.team4099.lib.units.base.inAttoseconds
import com.team4099.lib.units.base.inCentiseconds
import com.team4099.lib.units.base.inDeciseconds
import com.team4099.lib.units.base.inFemtoseconds
import com.team4099.lib.units.base.inHours
import com.team4099.lib.units.base.inMicroseconds
import com.team4099.lib.units.base.inMilliseconds
import com.team4099.lib.units.base.inMinutes
import com.team4099.lib.units.base.inNanoseconds
import com.team4099.lib.units.base.inPicoseconds
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.inYoctoseconds
import com.team4099.lib.units.base.inZeptoseconds
import com.team4099.lib.units.base.minutes
import com.team4099.lib.units.base.seconds
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals

/* Unit tests for Time units */
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
    assertEquals(minutes.inSeconds, minutes.inMinutes * SECONDS_PER_MINUTE, kEpsilon)
  }

  @Test
  fun testHoursToSeconds() {
    val hours = 4099.hours
    assertEquals(hours.inSeconds, hours.inHours * SECONDS_PER_HOUR, kEpsilon)
  }

  @Test
  fun testSecondsToDeciseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inDeciseconds, 1E1, kEpsilon)
  }

  @Test
  fun testSecondsToCentiseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inCentiseconds, 1E2, kEpsilon)
  }

  @Test
  fun testSecondsToMilliseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inMilliseconds, 1E3, kEpsilon)
  }

  @Test
  fun testSecondsToMicroseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inMicroseconds, 1E6, kEpsilon)
  }

  @Test
  fun testSecondsToNanoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inNanoseconds, 1E9, kDelta)
  }

  @Test
  fun testSecondsToPicoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inPicoseconds, 1E12, kDelta)
  }

  @Test
  fun testSecondsToFemtoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inFemtoseconds, 1E15, kSigma)
  }

  @Test
  fun testSecondsToAttoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inAttoseconds, 1E18, kSigma)
  }

  @Test
  fun testSecondsToZeptoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inZeptoseconds, 1E21, kAlpha)
  }

  @Test
  fun testSecondsToYoctoseconds() {
    val seconds = 1.seconds
    assertEquals(seconds.inYoctoseconds, 1E24, kBeta)
  }

  @Test
  fun testAddingSeconds() {
    val seconds1 = 9.seconds
    val seconds2 = 10.seconds
    assertEquals((seconds1 + seconds2).inSeconds, 19.0, kEpsilon)
  }

  @Test
  fun testSubtractingSeconds() {
    val seconds1 = 10.seconds
    val seconds2 = 9.seconds
    assertEquals((seconds1 - seconds2).inSeconds, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingSecondsByScalar() {
    val seconds = 2.seconds
    val scalar = 3.0
    assertEquals((seconds * scalar).inSeconds, 6.0, kEpsilon)
  }

  @Test
  fun testDividingSecondsByScalar() {
    val seconds = 6.seconds
    val scalar = 3
    assertEquals((seconds / scalar).inSeconds, 2.0, kEpsilon)
  }
}
