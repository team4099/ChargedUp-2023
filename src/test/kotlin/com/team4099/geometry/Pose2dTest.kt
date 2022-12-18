package com.team4099.geometry

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Transform2d
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test

class Pose2dTest {
  private val kEpsilon = 1E-9

  @Test
  fun testTransformBy() {
    val initial = Pose2d(Translation2d(1.0.meters, 2.0.meters), Rotation2d(45.0.degrees))
    val transformation = Transform2d(Translation2d(5.0.meters, 0.0.meters), Rotation2d(5.0.degrees))
    val transformed = initial.transformBy(transformation)
    assertAll(
      { assertEquals(1.0 + 5.0 / Math.sqrt(2.0), transformed.x.value, kEpsilon) },
      { assertEquals(2.0 + 5.0 / Math.sqrt(2.0), transformed.y.value, kEpsilon) },
      { assertEquals(50.0.degrees.value, transformed.theta.value, kEpsilon) }
    )
  }

  @Test
  fun testRelativeTo() {
    val initial = Pose2d(0.0.meters, 0.0.meters, Rotation2d(45.0.degrees))
    val last = Pose2d(5.0.meters, 5.0.meters, Rotation2d(45.0.degrees))
    val finalRelativeToInitial = last.relativeTo(initial)
    assertAll(
      { assertEquals(5.0 * Math.sqrt(2.0), finalRelativeToInitial.x.value, kEpsilon) },
      { assertEquals(0.0, finalRelativeToInitial.y.value, kEpsilon) },
      { assertEquals(0.0, finalRelativeToInitial.theta.value, kEpsilon) }
    )
  }

  @Test
  fun testEquality() {
    val one = Pose2d(0.0.meters, 5.0.meters, Rotation2d(43.0.degrees))
    val two = Pose2d(0.0.meters, 5.0.meters, Rotation2d(43.0.degrees))
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val one = Pose2d(0.0.meters, 5.0.meters, Rotation2d(43.0.degrees))
    val two = Pose2d(0.0.meters, 1.524.meters, Rotation2d(43.0.degrees))
    assertNotEquals(one, two)
  }

  @Test
  fun testMinus() {
    val initial = Pose2d(0.0.meters, 0.0.meters, Rotation2d(45.0.degrees))
    val last = Pose2d(5.0.meters, 5.0.meters, Rotation2d(45.0.degrees))
    val transform = last.minus(initial)
    assertAll(
      { assertEquals(5.0 * Math.sqrt(2.0), transform.m_translation.x.value, kEpsilon) },
      { assertEquals(0.0, transform.m_translation.y.value, kEpsilon) },
      { assertEquals(0.0, transform.m_rotation.theta.value, kEpsilon) }
    )
  }

  @Test
  fun testMultiply() {
    val rot = Rotation2d(45.0.degrees)
    val pose2d = Pose2d(5.0.meters, 5.0.meters, rot)
    assertEquals(pose2d * 3.0, Pose2d(15.0.meters, 15.0.meters, rot * 3.0))
  }

  @Test
  fun testDivide() {
    val rot = Rotation2d(45.0.degrees)
    val pose2d = Pose2d(10.0.meters, 10.0.meters, rot)
    assertEquals(pose2d / 2.0, Pose2d(5.0.meters, 5.0.meters, rot / 2.0))
  }
}
