package com.team4099

import com.team4099.lib.units.base.grams
import com.team4099.lib.units.base.inPounds
import junit.framework.TestCase.assertEquals
import org.junit.Test

class MassTests {
  private val kEpsilon = 1E-9

  @Test
  fun testGramsToPounds() {
    val weight = 4099.grams
    assertEquals(weight.inPounds, 9.036, kEpsilon)
  }
}
