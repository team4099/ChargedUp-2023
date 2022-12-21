package com.team4099.units

import com.team4099.lib.units.base.METERS_PER_FOOT
import com.team4099.lib.units.base.METERS_PER_INCH
import com.team4099.lib.units.base.METERS_PER_THOU
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inAttometers
import com.team4099.lib.units.base.inCentimeters
import com.team4099.lib.units.base.inDecameters
import com.team4099.lib.units.base.inDecimeters
import com.team4099.lib.units.base.inExameters
import com.team4099.lib.units.base.inFeet
import com.team4099.lib.units.base.inFemtometers
import com.team4099.lib.units.base.inGigameters
import com.team4099.lib.units.base.inHectometers
import com.team4099.lib.units.base.inInches
import com.team4099.lib.units.base.inKilometers
import com.team4099.lib.units.base.inMegameters
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inMicrometers
import com.team4099.lib.units.base.inMillimeters
import com.team4099.lib.units.base.inNanometers
import com.team4099.lib.units.base.inPetameters
import com.team4099.lib.units.base.inPicometers
import com.team4099.lib.units.base.inTerameters
import com.team4099.lib.units.base.inThou
import com.team4099.lib.units.base.inYoctometers
import com.team4099.lib.units.base.inYottameters
import com.team4099.lib.units.base.inZeptometers
import com.team4099.lib.units.base.inZetameters
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.thou
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/* Unit tests for Length units */
class LengthTest {
  private val kEpsilon = 1E-9

  @Test
  fun testMetersToInches() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inInches, lengthMeters.inMeters / METERS_PER_INCH, kEpsilon)
  }

  @Test
  fun testInchesToMeters() {
    val lengthInches = 4099.inches
    assertEquals(lengthInches.inMeters, lengthInches.inInches * METERS_PER_INCH, kEpsilon)
  }

  @Test
  fun testMetersToThou() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inThou, lengthMeters.inMeters / METERS_PER_THOU, kEpsilon)
  }

  @Test
  fun testThouToMeters() {
    val lengthThou = 4099.thou
    assertEquals(lengthThou.inMeters, lengthThou.inThou * METERS_PER_THOU, kEpsilon)
  }

  @Test
  fun testMetersToFeet() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inFeet, lengthMeters.inMeters / METERS_PER_FOOT, kEpsilon)
  }

  @Test
  fun testFeetToMeters() {
    val lengthFeet = 4099.feet
    assertEquals(lengthFeet.inMeters, lengthFeet.inFeet * METERS_PER_FOOT, kEpsilon)
  }

  @Test
  fun testMetersToYottameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inYottameters, 4.099E-21, kEpsilon)
  }

  @Test
  fun testMetersToZetameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inZetameters, 4.099E-18, kEpsilon)
  }

  @Test
  fun testMetersToExameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inExameters, 4.099E-15, kEpsilon)
  }

  @Test
  fun testMetersToPetameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inPetameters, 4.099E-12, kEpsilon)
  }

  @Test
  fun testMetersToTerameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inTerameters, 4.099E-9, kEpsilon)
  }

  @Test
  fun testMetersToGigameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inGigameters, 4.099E-6, kEpsilon)
  }

  @Test
  fun testMetersToMegameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inMegameters, 4.099E-3, kEpsilon)
  }

  @Test
  fun testMetersToKilometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inKilometers, 4.099E0, kEpsilon)
  }

  @Test
  fun testMetersToHectometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inHectometers, 4.099E1, kEpsilon)
  }

  @Test
  fun testMetersToDecameters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inDecameters, 4.099E2, kEpsilon)
  }

  @Test
  fun testMetersToDecimeters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inDecimeters, 4.099E4, kEpsilon)
  }

  @Test
  fun testMeterstoCentimeters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inCentimeters, 4.099E5, kEpsilon)
  }

  @Test
  fun testMeterstoMillimeters() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inMillimeters, 4.099E6, kEpsilon)
  }

  @Test
  fun testMeterstoMicrometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inMicrometers, 4.099E9, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoNanometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inNanometers, 4.099E12, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoPicometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inPicometers, 4.099E15, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoFemtometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inFemtometers, 4.099E18, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoAttometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inAttometers, 4.099E21, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoZeptometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inZeptometers, 4.099E24, kEpsilon)
  }

  // Unstable due to floating point errors
  @Test
  fun testMeterstoYoctometers() {
    val lengthMeters = 4099.meters
    assertEquals(lengthMeters.inYoctometers, 4.099E27, kEpsilon)
  }

  @Test
  fun testAddingMeters() {
    val g1 = 1.0.meters
    val g2 = 2.0.meters
    assertEquals((g1 + g2).inMeters, 3.0, kEpsilon)
  }

  @Test
  fun testSubtractingMeters() {
    val g1 = 1.0.meters
    val g2 = 2.0.meters
    assertEquals((g2 - g1).inMeters, 1.0, kEpsilon)
  }

  @Test
  fun testMultiplyingMetersByScalar() {
    val g1 = 1.0.meters
    assertEquals((g1 * 4099).inMeters, 4099.0, kEpsilon)
  }

  @Test
  fun testDividingMetersByScalar() {
    val g1 = 4099.0.meters
    assertEquals((g1 / 4099.0).inMeters, 1.0, kEpsilon)
  }
}
