package com.team4099

import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inGrams
import com.team4099.lib.units.base.inPounds
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class MassTest {
  private val kEpsilon = 1E-9

  @Test
  fun testGramsToPounds() {
    val weightGrams = 4099.grams
    assertEquals(weightGrams.inPounds, weightGrams.inGrams / 453.5924, kEpsilon)
  }
}
