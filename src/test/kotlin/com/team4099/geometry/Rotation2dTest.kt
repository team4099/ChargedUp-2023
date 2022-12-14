package com.team4099.geometry

import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.inRadians
import edu.wpi.first.math.geometry.Rotation2d.fromDegrees
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test

class Rotation2dTest {
  private val kEpsilon = 1E-9

  // floating point imprecision
  private val kDelta = 1E-3

  @Test
  fun testRotateByFromZero() {
    val zero = Rotation2d()
    val rotated = zero.rotateBy(Rotation2d(90.0.degrees))
    assertAll(
      { assertEquals(Math.PI / 2.0, rotated.theta.inRadians, kEpsilon) },
      { assertEquals(90.0, rotated.theta.inDegrees, kEpsilon) })
  }

  @Test
  fun testRotateByNonZero() {
    var rot = Rotation2d(90.0.degrees)
    rot = rot.plus(Rotation2d(30.0.degrees))
    assertEquals(120.0, rot.theta.inDegrees, kEpsilon)
  }

  @Test
  fun testMinus() {
    val rot1 = Rotation2d(70.0.degrees)
    val rot2 = Rotation2d(30.0.degrees)
    assertEquals(40.0, rot1.minus(rot2).theta.inDegrees, kEpsilon)
  }

  @Test
  fun testUnaryMinus() {
    val rot = Rotation2d(20.0.degrees)
    assertEquals(-20.0, rot.unaryMinus().theta.inDegrees, kEpsilon)
  }

  @Test
  fun testMultiply() {
    val rot = Rotation2d(10.0.degrees)
    assertEquals(30.0, rot.times(3.0).theta.inDegrees, kEpsilon)
    assertEquals(410.0 % 360.0, rot.times(41.0).theta.inDegrees, kEpsilon)
  }

  @Test
  fun testEquality() {
    var rot1 = Rotation2d(43.0.degrees)
    var rot2 = Rotation2d(43.0.degrees)
    assertEquals(rot1, rot2)
    rot1 = Rotation2d(-180.0.degrees)
    rot2 = Rotation2d(180.0.degrees)
    assertEquals(rot1, rot2)
  }

  @Test
  fun testInequality() {
    val rot1 = Rotation2d(43.0.degrees)
    val rot2 = Rotation2d(43.5.degrees)
    assertNotEquals(rot1, rot2)
  }

  @Test
  fun testInterpolate() {
    // 50 + (70 - 50) * 0.5 = 60
    var rot1 = Rotation2d(50.degrees)
    var rot2 = Rotation2d(70.degrees)
    var interpolated = rot1.interpolate(rot2, 0.5)
    if (interpolated != null) {
      assertEquals(60.0, interpolated.theta.inDegrees, kEpsilon)
    }

    // -160 minus half distance between 170 and -160 (15) = -175
    rot1 = Rotation2d(170.degrees)
    rot2 = Rotation2d(-160.degrees)
    interpolated = rot1.interpolate(rot2, 0.5)
    if (interpolated != null) {
      assertEquals(-175.0, interpolated.theta.inDegrees)
    }
  }
}
