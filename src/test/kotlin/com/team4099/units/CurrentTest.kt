package com.team4099.units

import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.inAmperes
import com.team4099.lib.units.base.inAttoamps
import com.team4099.lib.units.base.inCentiamps
import com.team4099.lib.units.base.inDecaamps
import com.team4099.lib.units.base.inDeciamps
import com.team4099.lib.units.base.inExaamps
import com.team4099.lib.units.base.inFemtoamps
import com.team4099.lib.units.base.inGigaamps
import com.team4099.lib.units.base.inHectoamps
import com.team4099.lib.units.base.inKiloamps
import com.team4099.lib.units.base.inMegaamps
import com.team4099.lib.units.base.inMicroamps
import com.team4099.lib.units.base.inMilliamps
import com.team4099.lib.units.base.inNanoamps
import com.team4099.lib.units.base.inPetaamps
import com.team4099.lib.units.base.inPicoamps
import com.team4099.lib.units.base.inTeraamps
import com.team4099.lib.units.base.inYoctoamps
import com.team4099.lib.units.base.inYottaamps
import com.team4099.lib.units.base.inZeptoamps
import com.team4099.lib.units.base.inZetaamps
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/* Unit tests class for Current.kt */
class CurrentTest {
  private val kEpsilon = 1E-9
  // Hacky way to do this. Consider removing support in the future for nanoamps and below due to
  // floating point imprecision
  private val kDelta = 1E-3
  private val kSigma = 1E6
  private val kAlpha = 1E9
  private val kBeta = 1E12

  @Test
  fun testAmpsToYottaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inYottaamps, 4.099E-21, kEpsilon)
  }

  @Test
  fun testAmpsToZetaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inZetaamps, 4.099E-18, kEpsilon)
  }

  @Test
  fun testAmpsToExaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inExaamps, 4.099E-15, kEpsilon)
  }

  @Test
  fun testAmpsToPetaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inPetaamps, 4.099E-12, kEpsilon)
  }

  @Test
  fun testAmpsToTeraamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inTeraamps, 4.099E-9, kEpsilon)
  }

  @Test
  fun testAmpsToGigaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inGigaamps, 4.099E-6, kEpsilon)
  }

  @Test
  fun testAmpsToMegaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inMegaamps, 4.099E-3, kEpsilon)
  }

  @Test
  fun testAmpsToKiloamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inKiloamps, 4.099E0, kEpsilon)
  }

  @Test
  fun testAmpsToHectoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inHectoamps, 4.099E1, kEpsilon)
  }

  @Test
  fun testAmpsToDecaamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inDecaamps, 4.099E2, kEpsilon)
  }

  @Test
  fun testAmpsToDeciamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inDeciamps, 4.099E4, kEpsilon)
  }

  @Test
  fun testAmpstoCentiamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inCentiamps, 4.099E5, kEpsilon)
  }

  @Test
  fun testAmpstoMilliamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inMilliamps, 4.099E6, kEpsilon)
  }

  @Test
  fun testAmpstoMicroamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inMicroamps, 4.099E9, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoNanoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inNanoamps, 4.099E12, kDelta)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoPicoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inPicoamps, 4.099E15, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoFemtoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inFemtoamps, 4.099E18, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoAttoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inAttoamps, 4.099E21, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoZeptoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inZeptoamps, 4.099E24, kAlpha)
  }

  // Unstable due to floating point errors
  @Test
  fun testAmpstoYoctoamps() {
    val currentAmps = 4099.amps
    assertEquals(currentAmps.inYoctoamps, 4.099E27, kBeta)
  }

  @Test
  fun testAddingAmps() {
    val g1 = 1.0.amps
    val g2 = 2.0.amps
    assertEquals((g1 + g2).inAmperes, 3.0, kEpsilon)
  }

  @Test
  fun testSubtractingAmps() {
    val g1 = 1.0.amps
    val g2 = 2.0.amps
    assertEquals((g2 - g1).inAmperes, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingAmpsByScalar() {
    val g1 = 1.0.amps
    assertEquals((g1 * 4099).inAmperes, 4099.0, kEpsilon)
  }

  @Test
  fun testDividingAmpsByScalar() {
    val g1 = 4099.0.amps
    assertEquals((g1 / 4099.0).inAmperes, 1.0, kEpsilon)
  }
}
