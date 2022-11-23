package com.team4099

import com.team4099.lib.units.base.GRAMS_PER_POUND
import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inAttograms
import com.team4099.lib.units.base.inCentigrams
import com.team4099.lib.units.base.inDecagrams
import com.team4099.lib.units.base.inDecigrams
import com.team4099.lib.units.base.inExagrams
import com.team4099.lib.units.base.inFemtograms
import com.team4099.lib.units.base.inGigagrams
import com.team4099.lib.units.base.inGrams
import com.team4099.lib.units.base.inHectograms
import com.team4099.lib.units.base.inKilograms
import com.team4099.lib.units.base.inMegagrams
import com.team4099.lib.units.base.inMicrograms
import com.team4099.lib.units.base.inMilligrams
import com.team4099.lib.units.base.inNanograms
import com.team4099.lib.units.base.inPetagrams
import com.team4099.lib.units.base.inPicograms
import com.team4099.lib.units.base.inPounds
import com.team4099.lib.units.base.inTeragrams
import com.team4099.lib.units.base.inYoctograms
import com.team4099.lib.units.base.inYottagrams
import com.team4099.lib.units.base.inZeptograms
import com.team4099.lib.units.base.inZetagrams
import com.team4099.lib.units.base.pounds
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/* Unit tests class for Mass.kt */
class MassTest {
  private val kEpsilon = 1E-9
  // Hacky way to do this. Consider removing support in the future for nanograms and below due to
  // floating point imprecision
  private val kDelta = 1E-3
  private val kSigma = 1E6
  private val kAlpha = 1E9
  private val kBeta = 1E12

  @Test
  fun testGramsToPounds() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inPounds, weightGrams.inGrams / GRAMS_PER_POUND, kEpsilon)
  }

  @Test
  fun testPoundsToGrams() {
    val weightPounds = 4099.pounds
    assertEquals(weightPounds.inGrams, weightPounds.inPounds * GRAMS_PER_POUND, kEpsilon)
  }

  @Test
  fun testGramsToYottagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inYottagrams, 4.099E-21, kEpsilon)
  }

  @Test
  fun testGramsToZetagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inZetagrams, 4.099E-18, kEpsilon)
  }

  @Test
  fun testGramsToExagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inExagrams, 4.099E-15, kEpsilon)
  }

  @Test
  fun testGramsToPetagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inPetagrams, 4.099E-12, kEpsilon)
  }

  @Test
  fun testGramsToTeragrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inTeragrams, 4.099E-9, kEpsilon)
  }

  @Test
  fun testGramsToGigagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inGigagrams, 4.099E-6, kEpsilon)
  }

  @Test
  fun testGramsToMegagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inMegagrams, 4.099E-3, kEpsilon)
  }

  @Test
  fun testGramsToKilograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inKilograms, 4.099E0, kEpsilon)
  }

  @Test
  fun testGramsToHectograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inHectograms, 4.099E1, kEpsilon)
  }

  @Test
  fun testGramsToDecagrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inDecagrams, 4.099E2, kEpsilon)
  }

  @Test
  fun testGramsToDecigrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inDecigrams, 4.099E4, kEpsilon)
  }

  @Test
  fun testGramstoCentigrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inCentigrams, 4.099E5, kEpsilon)
  }

  @Test
  fun testGramstoMilligrams() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inMilligrams, 4.099E6, kEpsilon)
  }

  @Test
  fun testGramstoMicrograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inMicrograms, 4.099E9, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoNanograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inNanograms, 4.099E12, kDelta)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoPicograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inPicograms, 4.099E15, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoFemtograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inFemtograms, 4.099E18, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoAttograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inAttograms, 4.099E21, kSigma)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoZeptograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inZeptograms, 4.099E24, kAlpha)
  }

  // Unstable due to floating point errors
  @Test
  fun testGramstoYoctograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inYoctograms, 4.099E27, kBeta)
  }

  @Test
  fun testAddingGrams() {
    val g1 = 1.0.grams
    val g2 = 2.0.grams
    assertEquals((g1 + g2).inGrams, 3.0, kEpsilon)
  }

  @Test
  fun testSubtractingGrams() {
    val g1 = 1.0.grams
    val g2 = 2.0.grams
    assertEquals((g2 - g1).inGrams, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingGramsByScalar() {
    val g1 = 1.0.grams
    assertEquals((g1 * 4099).inGrams, 4099.0, kEpsilon)
  }
}
