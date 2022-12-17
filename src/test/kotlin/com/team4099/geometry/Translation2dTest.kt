package com.team4099.geometry

import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test
import kotlin.math.hypot
import kotlin.math.sqrt

internal class Translation2dTest {
  private val kEpsilon = 1E-9

  @Test
  fun testSum() {
    val one = Translation2d(1.0.meters, 3.0.meters)
    val two = Translation2d(2.0.meters, 5.0.meters)
    val sum = one.plus(two)
    assertAll(
      { assertEquals(3.0, sum.x.inMeters, kEpsilon) },
      { assertEquals(8.0, sum.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testDifference() {
    val one = Translation2d(1.0.meters, 3.0.meters)
    val two = Translation2d(2.0.meters, 5.0.meters)
    val difference = one.minus(two)
    assertAll(
      { assertEquals(-1.0, difference.x.inMeters, kEpsilon) },
      { assertEquals(-2.0, difference.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testRotateBy() {
    val another = Translation2d(3.0.meters, 0.0.meters)
    val rotated = another.rotateBy(Rotation2d(90.0.degrees))
    assertAll(
      { assertEquals(0.0, rotated.x.inMeters, kEpsilon) },
      { assertEquals(3.0, rotated.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testMultiplication() {
    val original = Translation2d(3.0.meters, 5.0.meters)
    val mult = original * 3.0
    assertAll(
      { assertEquals(9.0, mult.x.inMeters, kEpsilon) },
      { assertEquals(15.0, mult.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testDivision() {
    val original = Translation2d(3.0.meters, 5.0.meters)
    val div = original.div(2.0)
    assertAll(
      { assertEquals(1.5, div.x.inMeters, kEpsilon) },
      { assertEquals(2.5, div.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testNorm() {
    val one = Translation2d(3.0.meters, 5.0.meters)
    assertEquals(hypot(3.0, 5.0), one.magnitude, kEpsilon)
  }

  // Need to implement distance method
  @Test
  fun testDistance() {
    val one = Translation2d(1.meters, 1.meters)
    val two = Translation2d(6.meters, 6.meters)
    assertEquals(5.0 * sqrt(2.0), one.minus(two).magnitude, kEpsilon)
  }

  @Test
  fun testUnaryMinus() {
    val original = Translation2d((-4.5).meters, 7.meters)
    val inverted = original.unaryMinus()
    assertAll(
      { assertEquals(4.5, inverted.x.inMeters, kEpsilon) },
      { assertEquals(-7.0, inverted.y.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testEquality() {
    val one = Translation2d(9.meters, 5.5.meters)
    val two = Translation2d(9.meters, 5.5.meters)
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val one = Translation2d(9.meters, 5.5.meters)
    val two = Translation2d(9.meters, 5.7.meters)
    assertNotEquals(one, two)
  }
  /*
  @Test
  fun testPolarConstructor() {
    val one = Translation2d(sqrt(2.0).meters, 45.0.degrees))
    val two = Translation2d(2.meters, 60.degrees)
    assertAll(
      { assertEquals(1.0, one.getX(), kEpsilon) },
      { assertEquals(1.0, one.getY(), kEpsilon) },
      { assertEquals(1.0, two.getX(), kEpsilon) }
    , {
      assertEquals(
        Math.sqrt(3.0),
        two.getY(),
        kEpsilon
      )
    })
  }

   */
}
