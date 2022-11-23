package com.team4099

import com.team4099.lib.units.base.GRAMS_PER_POUND
import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inCentigrams
import com.team4099.lib.units.base.inDecagrams
import com.team4099.lib.units.base.inDecigrams
import com.team4099.lib.units.base.inExagrams
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
import com.team4099.lib.units.base.inYottagrams
import com.team4099.lib.units.base.inZetagrams
import com.team4099.lib.units.base.pounds
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class MassTest {
  private val kEpsilon = 1E-9
  private val kDelta = 1E-3

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
  fun testGramstoCentigrams(){
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inCentigrams, 4.099E5, kEpsilon)
  }

  @Test
  fun testGramstoMilligrams(){
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inMilligrams, 4.099E6, kEpsilon)
  }

  @Test
  fun testGramstoMicrograms(){
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inMicrograms, 4.099E9, kEpsilon)
  }

  @Test
  fun testGramstoNanograms(){
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inNanograms, 4.099E12, kDelta)
  }

  @Test
  fun testGramstoPicograms(){
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inPicograms, 4.099E15, kDelta)
  }

}
