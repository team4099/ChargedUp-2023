package com.team4099.geometry

import com.team4099.lib.geometry.Pose3d
import com.team4099.lib.geometry.Rotation3d
import com.team4099.lib.geometry.Twist3d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.VecBuilder
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test

class Twist3dTest {
  @Test
  fun testInit() {
    val one = Twist3d()
    val two = Twist3d(0.0.meters, 0.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 0.0.radians)
    assertEquals(one, two)
  }

  @Test
  fun testStraightX() {
    val straight =
      Twist3d(5.0.meters, 0.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 0.0.radians)
    val straightPose = Pose3d().exp(straight)
    val expected = Pose3d(5.0.meters, 0.0.meters, 0.0.meters, Rotation3d())
    assertEquals(expected, straightPose)
  }

  @Test
  fun testStraightY() {
    val straight =
      Twist3d(0.0.meters, 5.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 0.0.radians)
    val straightPose = Pose3d().exp(straight)
    val expected = Pose3d(0.0.meters, 5.0.meters, 0.0.meters, Rotation3d())
    assertEquals(expected, straightPose)
  }

  @Test
  fun testStraightZ() {
    val straight =
      Twist3d(0.0.meters, 0.0.meters, 5.0.meters, 0.0.radians, 0.0.radians, 0.0.radians)
    val straightPose = Pose3d().exp(straight)
    val expected = Pose3d(0.0.meters, 0.0.meters, 5.0.meters, Rotation3d())
    assertEquals(expected, straightPose)
  }

  @Test
  fun testQuarterCirle() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val quarterCircle =
      Twist3d(
        (5.0 / 2.0 * Math.PI).meters,
        0.0.meters,
        0.0.meters,
        0.0.radians,
        0.0.radians,
        (Math.PI / 2.0).radians
      )
    val quarterCirclePose = Pose3d().exp(quarterCircle)
    val expected = Pose3d(5.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 90.0.degrees))
    assertEquals(expected, quarterCirclePose)
  }

  @Test
  fun testDiagonalNoDtheta() {
    val diagonal =
      Twist3d(2.0.meters, 2.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 0.0.radians)
    val diagonalPose = Pose3d().exp(diagonal)
    val expected = Pose3d(2.0.meters, 2.0.meters, 0.0.meters, Rotation3d())
    assertEquals(expected, diagonalPose)
  }

  @Test
  fun testEquality() {
    val one = Twist3d(5.0.meters, 1.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 3.0.radians)
    val two = Twist3d(5.0.meters, 1.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 3.0.radians)
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val one = Twist3d(5.0.meters, 1.0.meters, 0.0.meters, 0.0.radians, 0.0.radians, 3.0.radians)
    val two = Twist3d(5.0.meters, 1.2.meters, 0.0.meters, 0.0.radians, 0.0.radians, 3.0.radians)
    assertNotEquals(one, two)
  }

  @Test
  fun testPose3dLogX() {
    val start = Pose3d()
    val end =
      Pose3d(
        0.0.meters, 5.0.meters, 5.0.meters, Rotation3d(90.0.degrees, 0.0.radians, 0.0.radians)
      )
    val twist = start.log(end)
    val expected =
      Twist3d(
        0.0.meters,
        (5.0 / 2.0 * Math.PI).meters,
        0.0.meters,
        90.0.degrees,
        0.0.radians,
        0.0.radians
      )
    assertEquals(expected, twist)

    // Make sure computed twist gives back original end pose
    val reapplied = start.exp(twist)
    assertEquals(end, reapplied)
  }

  @Test
  fun testPose3dLogY() {
    val start = Pose3d()
    val end =
      Pose3d(5.0.meters, 0.0.meters, 5.0.meters, Rotation3d(0.0.radians, 90.degrees, 0.0.radians))
    val twist = start.log(end)
    val expected =
      Twist3d(
        0.0.meters,
        0.0.meters,
        (5.0 / 2.0 * Math.PI).meters,
        0.0.radians,
        (Math.PI / 2.0).radians,
        0.0.radians
      )
    assertEquals(expected, twist)

    // Make sure computed twist gives back original end pose
    val reapplied = start.exp(twist)
    assertEquals(end, reapplied)
  }

  @Test
  fun testPose3dLogZ() {
    val start = Pose3d()
    val end =
      Pose3d(
        5.0.meters, 5.0.meters, 0.0.meters, Rotation3d(0.0.radians, 0.0.radians, 90.0.degrees)
      )
    val twist = start.log(end)
    val expected =
      Twist3d(
        (5.0 / 2.0 * Math.PI).meters,
        0.0.meters,
        0.0.meters,
        0.0.radians,
        0.0.radians,
        (Math.PI / 2.0).radians
      )
    assertEquals(expected, twist)

    // Make sure computed twist gives back original end pose
    val reapplied = start.exp(twist)
    assertEquals(end, reapplied)
  }
}
