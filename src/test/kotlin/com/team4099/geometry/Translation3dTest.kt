package com.team4099.geometry

import com.team4099.lib.geometry.Rotation3d
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.geometry.Translation3d
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import edu.wpi.first.math.VecBuilder
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertAll
import kotlin.math.sqrt

internal class Translation3dTest {
  private val kEpsilon = 1E-9

  @Test
  fun testSum() {
    val one = Translation3d(1.0.meters, 3.0.meters, 5.0.meters)
    val two = Translation3d(2.0.meters, 5.0.meters, 8.0.meters)
    val sum = one.plus(two)
    assertAll(
      { assertEquals(3.0, sum.x.inMeters, kEpsilon) },
      { assertEquals(8.0, sum.y.inMeters, kEpsilon) },
      { assertEquals(13.0, sum.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testDifference() {
    val one = Translation3d(1.0.meters, 3.0.meters, 5.0.meters)
    val two = Translation3d(2.0.meters, 5.0.meters, 8.0.meters)
    val difference = one.minus(two)
    assertAll(
      { assertEquals(-1.0, difference.x.inMeters, kEpsilon) },
      { assertEquals(-2.0, difference.y.inMeters, kEpsilon) },
      { assertEquals(-3.0, difference.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testRotateBy() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val translation = Translation3d(1.0.meters, 2.0.meters, 3.0.meters)
    val rotated1 = translation.rotateBy(Rotation3d(xAxis, 90.degrees))
    assertAll(
      { assertEquals(1.0, rotated1.x.inMeters, kEpsilon) },
      { assertEquals(-3.0, rotated1.y.inMeters, kEpsilon) },
      { assertEquals(2.0, rotated1.z.inMeters, kEpsilon) }
    )
    val rotated2 = translation.rotateBy(Rotation3d(yAxis, 90.0.degrees))
    assertAll(
      { assertEquals(3.0, rotated2.x.inMeters, kEpsilon) },
      { assertEquals(2.0, rotated2.y.inMeters, kEpsilon) },
      { assertEquals(-1.0, rotated2.z.inMeters, kEpsilon) }
    )
    val rotated3 = translation.rotateBy(Rotation3d(zAxis, 90.0.degrees))
    assertAll(
      { assertEquals(-2.0, rotated3.x.inMeters, kEpsilon) },
      { assertEquals(1.0, rotated3.y.inMeters, kEpsilon) },
      { assertEquals(3.0, rotated3.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testToTranslation2d() {
    val translation = Translation3d(1.0.meters, 2.0.meters, 3.0.meters)
    val expected = Translation2d(1.0.meters, 2.0.meters)
    assertEquals(expected, translation.toTranslation2d())
  }

  @Test
  fun testMultiplication() {
    val original = Translation3d(3.0.meters, 5.0.meters, 7.0.meters)
    val mult = original.times(3.0)
    assertAll(
      { assertEquals(9.0, mult.x.inMeters, kEpsilon) },
      { assertEquals(15.0, mult.y.inMeters, kEpsilon) },
      { assertEquals(21.0, mult.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testDivision() {
    val original = Translation3d(3.0.meters, 5.0.meters, 7.0.meters)
    val div = original.div(2.0)
    assertAll(
      { assertEquals(1.5, div.x.inMeters, kEpsilon) },
      { assertEquals(2.5, div.y.inMeters, kEpsilon) },
      { assertEquals(3.5, div.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testNorm() {
    val one = Translation3d(3.0.meters, 5.0.meters, 7.0.meters)
    assertEquals(sqrt(83.0), one.norm.inMeters, kEpsilon)
  }

  @Test
  fun testDistance() {
    val one = Translation3d(1.0.meters, 1.0.meters, 1.0.meters)
    val two = Translation3d(6.0.meters, 6.0.meters, 6.0.meters)
    assertEquals(5.0 * sqrt(3.0), one.getDistance(two).inMeters, kEpsilon)
  }

  @Test
  fun testUnaryMinus() {
    val original = Translation3d((-4.5).meters, 7.0.meters, 9.0.meters)
    val inverted = original.unaryMinus()
    assertAll(
      { assertEquals(4.5, inverted.x.inMeters, kEpsilon) },
      { assertEquals(-7.0, inverted.y.inMeters, kEpsilon) },
      { assertEquals(-9.0, inverted.z.inMeters, kEpsilon) }
    )
  }

  @Test
  fun testEquality() {
    val one = Translation3d(9.meters, 5.5.meters, 3.5.meters)
    val two = Translation3d(9.meters, 5.5.meters, 3.5.meters)
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val one = Translation3d(9.meters, 5.5.meters, 3.5.meters)
    val two = Translation3d(9.meters, 5.7.meters, 3.5.meters)
    assertNotEquals(one, two)
  }

  @Test
  fun testPolarConstructor() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val one = Translation3d(sqrt(2.0).meters, Rotation3d(zAxis, 45.0.degrees))
    val two = Translation3d(2.0.meters, Rotation3d(zAxis, 60.0.degrees))
    assertAll(
      { assertEquals(1.0, one.x.inMeters, kEpsilon) },
      { assertEquals(1.0, one.y.inMeters, kEpsilon) },
      { assertEquals(0.0, one.z.inMeters, kEpsilon) },
      { assertEquals(1.0, two.x.inMeters, kEpsilon) },
      { assertEquals(sqrt(3.0), two.y.inMeters, kEpsilon) },
      { assertEquals(0.0, two.z.inMeters, kEpsilon) }
    )
  }
}
