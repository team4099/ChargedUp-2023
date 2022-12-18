package com.team4099.geometry

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Twist2d
import com.team4099.lib.geometry.Twist3d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.radians
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test

class Twist2dTest {
  @Test
  fun testInit() {
    val one = Twist2d()
    val two = Twist2d(0.0.meters, 0.0.meters, 0.0.radians)
    assertEquals(one, two)
  }

  // need to individually test each variable
  @Test
  fun testStraight() {
    val straight = Twist2d(5.0.meters, 0.0.meters, 0.0.radians)
    val straightPose = Pose2d().exp(straight)
    val expected = Pose2d(5.0.meters, 0.0.meters, Rotation2d())
    assertEquals(expected, straightPose)
  }

  @Test
  fun testQuarterCircle() {
    val quarterCircle = Twist2d((5.0 / 2.0 * Math.PI).meters, 0.0.meters, (Math.PI / 2.0).radians)
    val quarterCirclePose = Pose2d().exp(quarterCircle)
    val expected = Pose2d(5.0.meters, 5.0.meters, Rotation2d(90.0.degrees))
    assertEquals(expected, quarterCirclePose)
  }

  @Test
  fun testDiagonalNoDtheta() {
    val diagonal = Twist2d(2.0.meters, 2.0.meters, 0.0.radians)
    val diagonalPose = Pose2d().exp(diagonal)
    val expected = Pose2d(2.0.meters, 2.0.meters, Rotation2d())
    assertEquals(expected, diagonalPose)
  }

  @Test
  fun testEquality() {
    val one = Twist2d(5.meters, 1.meters, 3.radians)
    val two = Twist2d(5.meters, 1.meters, 3.radians)
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val one = Twist2d(5.meters, 1.0.meters, 3.radians)
    val two = Twist2d(5.meters, 1.2.meters, 3.radians)
    assertNotEquals(one, two)
  }

  @Test
  fun testPose2dLog() {
    val start = Pose2d()
    val end = Pose2d(5.0.meters, 5.0.meters, Rotation2d(90.0.degrees))
    val twist = start.log(end)
    val expected = Twist2d((5.0 / 2.0 * Math.PI).meters, 0.0.meters, (Math.PI / 2.0).radians)
    assertEquals(expected, twist)

    // Make sure computed twist gives back original end pose
    val reapplied = start.exp(twist)
    assertEquals(end, reapplied)
  }
}
