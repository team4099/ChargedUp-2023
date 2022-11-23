package com.team4099

import com.team4099.lib.units.base.GRAMS_PER_POUND
import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inGrams
import com.team4099.lib.units.base.inKilograms
import com.team4099.lib.units.base.inPounds
import com.team4099.lib.units.base.pounds
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class MassTest {
  private val kEpsilon = 1E-9

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
  fun testGramsToKilograms() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inKilograms, 4.099, kEpsilon)
  }
}
