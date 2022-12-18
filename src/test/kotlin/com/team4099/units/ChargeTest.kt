package com.team4099.units

import com.team4099.lib.units.derived.coulombs
import com.team4099.lib.units.derived.inAttocoulombs
import com.team4099.lib.units.derived.inCenticoulombs
import com.team4099.lib.units.derived.inCoulombs
import com.team4099.lib.units.derived.inDecacoulombs
import com.team4099.lib.units.derived.inDecicoulombs
import com.team4099.lib.units.derived.inExacoulombs
import com.team4099.lib.units.derived.inFemtocoulombs
import com.team4099.lib.units.derived.inGigacoulombs
import com.team4099.lib.units.derived.inHectocoulombs
import com.team4099.lib.units.derived.inKilocoulombs
import com.team4099.lib.units.derived.inMegacoulombs
import com.team4099.lib.units.derived.inMicrocoulombs
import com.team4099.lib.units.derived.inMillicoulombs
import com.team4099.lib.units.derived.inNanocoulombs
import com.team4099.lib.units.derived.inPetacoulombs
import com.team4099.lib.units.derived.inPicocoulombs
import com.team4099.lib.units.derived.inTeracoulombs
import com.team4099.lib.units.derived.inYoctocoulombs
import com.team4099.lib.units.derived.inYottacoulombs
import com.team4099.lib.units.derived.inZeptocoulombs
import com.team4099.lib.units.derived.inZetacoulombs
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

/* Unit tests for Charge units */
class ChargeTest {
  private val kEpsilon = 1E-9

  @Test
  fun testCoulombsToYottacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inYottacoulombs, 4.099E-21, kEpsilon)
  }

  @Test
  fun testCoulombsToZetacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inZetacoulombs, 4.099E-18, kEpsilon)
  }

  @Test
  fun testCoulombsToExacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inExacoulombs, 4.099E-15, kEpsilon)
  }

  @Test
  fun testCoulombsToPetacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inPetacoulombs, 4.099E-12, kEpsilon)
  }

  @Test
  fun testCoulombsToTeracoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inTeracoulombs, 4.099E-9, kEpsilon)
  }

  @Test
  fun testCoulombsToGigacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inGigacoulombs, 4.099E-6, kEpsilon)
  }

  @Test
  fun testCoulombsToMegacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inMegacoulombs, 4.099E-3, kEpsilon)
  }

  @Test
  fun testCoulombsToKilocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inKilocoulombs, 4.099E0, kEpsilon)
  }

  @Test
  fun testCoulombsToHectocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inHectocoulombs, 4.099E1, kEpsilon)
  }

  @Test
  fun testCoulombsToDecacoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inDecacoulombs, 4.099E2, kEpsilon)
  }

  @Test
  fun testCoulombsToDecicoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inDecicoulombs, 4.099E4, kEpsilon)
  }

  @Test
  fun testCoulombstoCenticoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inCenticoulombs, 4.099E5, kEpsilon)
  }

  @Test
  fun testCoulombstoMillicoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inMillicoulombs, 4.099E6, kEpsilon)
  }

  @Test
  fun testCoulombstoMicrocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inMicrocoulombs, 4.099E9, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoNanocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inNanocoulombs, 4.099E12, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoPicocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inPicocoulombs, 4.099E15, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoFemtocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inFemtocoulombs, 4.099E18, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoAttocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inAttocoulombs, 4.099E21, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoZeptocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inZeptocoulombs, 4.099E24, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testCoulombstoYoctocoulombs() {
    val chargeCoulombs = 4099.coulombs
    Assertions.assertEquals(chargeCoulombs.inYoctocoulombs, 4.099E27, kEpsilon)
  }

  @Test
  fun testAddingCoulombs() {
    val g1 = 1.0.coulombs
    val g2 = 2.0.coulombs
    Assertions.assertEquals((g1 + g2).inCoulombs, 3.0, kEpsilon)
  }

  @Test
  fun testSubtractingCoulombs() {
    val g1 = 1.0.coulombs
    val g2 = 2.0.coulombs
    Assertions.assertEquals((g2 - g1).inCoulombs, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingCoulombsByScalar() {
    val g1 = 1.0.coulombs
    Assertions.assertEquals((g1 * 4099).inCoulombs, 4099.0, kEpsilon)
  }

  @Test
  fun testDividingCoulombsByScalar() {
    val g1 = 4099.0.coulombs
    Assertions.assertEquals((g1 / 4099.0).inCoulombs, 1.0, kEpsilon)
  }
}
