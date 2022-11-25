package com.team4099

import com.team4099.lib.units.derived.inAttojoules
import com.team4099.lib.units.derived.inCentijoules
import com.team4099.lib.units.derived.inDecajoules
import com.team4099.lib.units.derived.inDecijoules
import com.team4099.lib.units.derived.inExajoules
import com.team4099.lib.units.derived.inFemtojoules
import com.team4099.lib.units.derived.inGigajoules
import com.team4099.lib.units.derived.inHectojoules
import com.team4099.lib.units.derived.inJoules
import com.team4099.lib.units.derived.inKilojoules
import com.team4099.lib.units.derived.inMegajoules
import com.team4099.lib.units.derived.inMicrojoules
import com.team4099.lib.units.derived.inMillijoules
import com.team4099.lib.units.derived.inNanojoules
import com.team4099.lib.units.derived.inPetajoules
import com.team4099.lib.units.derived.inPicojoules
import com.team4099.lib.units.derived.inTerajoules
import com.team4099.lib.units.derived.inYoctojoules
import com.team4099.lib.units.derived.inYottajoules
import com.team4099.lib.units.derived.inZeptojoules
import com.team4099.lib.units.derived.inZetajoules
import com.team4099.lib.units.derived.joules
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class EnergyTest {
  private val kEpsilon = 1E-9
  // Hacky way to do this. Consider removing support in the future for nanojoules and below due to
  // floating point imprecision
  private val kDelta = 1E-3
  private val kSigma = 1E6
  private val kAlpha = 1E9
  private val kBeta = 1E12

  @Test
  fun testJoulesToYottajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inYottajoules, 4.099E-21, kEpsilon)
  }

  @Test
  fun testJoulesToZetajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inZetajoules, 4.099E-18, kEpsilon)
  }

  @Test
  fun testJoulesToExajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inExajoules, 4.099E-15, kEpsilon)
  }

  @Test
  fun testJoulesToPetajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inPetajoules, 4.099E-12, kEpsilon)
  }

  @Test
  fun testJoulesToTerajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inTerajoules, 4.099E-9, kEpsilon)
  }

  @Test
  fun testJoulesToGigajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inGigajoules, 4.099E-6, kEpsilon)
  }

  @Test
  fun testJoulesToMegajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inMegajoules, 4.099E-3, kEpsilon)
  }

  @Test
  fun testJoulesToKilojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inKilojoules, 4.099E0, kEpsilon)
  }

  @Test
  fun testJoulesToHectojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inHectojoules, 4.099E1, kEpsilon)
  }

  @Test
  fun testJoulesToDecajoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inDecajoules, 4.099E2, kEpsilon)
  }

  @Test
  fun testJoulesToDecijoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inDecijoules, 4.099E4, kEpsilon)
  }

  @Test
  fun testJoulestoCentijoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inCentijoules, 4.099E5, kEpsilon)
  }

  @Test
  fun testJoulestoMillijoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inMillijoules, 4.099E6, kEpsilon)
  }

  @Test
  fun testJoulestoMicrojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inMicrojoules, 4.099E9, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoNanojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inNanojoules, 4.099E12, kDelta)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoPicojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inPicojoules, 4.099E15, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoFemtojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inFemtojoules, 4.099E18, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoAttojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inAttojoules, 4.099E21, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoZeptojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inZeptojoules, 4.099E24, kAlpha)
  }

  // Unstable due to floating point errors
  @Test
  fun testJoulestoYoctojoules() {
    val energyJoules = 4099.joules
    Assertions.assertEquals(energyJoules.inYoctojoules, 4.099E27, kBeta)
  }

  @Test
  fun testAddingJoules() {
    val e1 = 1.0.joules
    val e2 = 2.0.joules
    Assertions.assertEquals((e1 + e2).inJoules, 3.0, kEpsilon)
  }

  @Test
  fun testSubtractingJoules() {
    val e1 = 1.0.joules
    val e2 = 2.0.joules
    Assertions.assertEquals((e2 - e1).inJoules, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingJoulesByScalar() {
    val e1 = 1.0.joules
    Assertions.assertEquals((e1 * 4099).inJoules, 4099.0, kEpsilon)
  }

  @Test
  fun testDividingJoulesByScalar() {
    val e1 = 4099.0.joules
    Assertions.assertEquals((e1 / 4099.0).inJoules, 1.0, kEpsilon)
  }
}
