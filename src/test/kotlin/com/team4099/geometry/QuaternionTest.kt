package com.team4099.geometry

import com.team4099.lib.geometry.Quaternion
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

internal class QuaternionTest {
  @Test
  fun testInit() {
    // Identity
    val q1 = Quaternion()
    assertEquals(1.0, q1.w.value)
    assertEquals(0.0, q1.x.value)
    assertEquals(0.0, q1.y.value)
    assertEquals(0.0, q1.z.value)

    // Normalized
    val q2 = Quaternion(0.5.radians, 0.5, 0.5, 0.5)
    assertEquals(0.5, q2.w.value)
    assertEquals(0.5, q2.x.value)
    assertEquals(0.5, q2.y.value)
    assertEquals(0.5, q2.z.value)

    // Unnormalized
    var q3 = Quaternion(0.75.radians, 0.3, 0.4, 0.5)
    assertEquals(0.75, q3.w.value)
    assertEquals(0.3, q3.x.value)
    assertEquals(0.4, q3.y.value)
    assertEquals(0.5, q3.z.value)
    q3 = q3.normalize()
    val norm = Math.sqrt(0.75 * 0.75 + 0.3 * 0.3 + 0.4 * 0.4 + 0.5 * 0.5)
    assertEquals(0.75 / norm, q3.w.value)
    assertEquals(0.3 / norm, q3.x.value)
    assertEquals(0.4 / norm, q3.y.value)
    assertEquals(0.5 / norm, q3.z.value)
    // may need to switch from radians to degrees
    assertEquals(
      1.0,
      (q3.w.inRadians * q3.w.inRadians) +
        (q3.x.value * q3.x.value) +
        q3.y.value * q3.y.value +
        q3.z.value * q3.z.value
    )
  }

  @Test
  fun testTimes() {
    // 90° CCW rotations around each axis
    val c = Math.cos(90.0.degrees.inRadians / 2.0)
    val s = Math.sin(90.0.degrees.inRadians / 2.0)
    val xRot = Quaternion(c.radians, s, 0.0, 0.0)
    val yRot = Quaternion(c.radians, 0.0, s, 0.0)
    val zRot = Quaternion(c.radians, 0.0, 0.0, s)

    // 90° CCW X rotation, 90° CCW Y rotation, and 90° CCW Z rotation should
    // produce a 90° CCW Y rotation
    val expected: Quaternion = yRot
    var actual = zRot.times(yRot).times(xRot)
    assertEquals(expected.w.inDegrees, actual.w.inDegrees, 1e-9)
    assertEquals(expected.x.value, actual.x.value, 1e-9)
    assertEquals(expected.y.value, actual.y.value, 1e-9)
    assertEquals(expected.z.value, actual.z.value, 1e-9)

    // Identity
    val q =
      Quaternion(
        0.72760687510899891.radians,
        0.29104275004359953,
        0.38805700005813276,
        0.48507125007266594
      )
    actual = q.times(q.inverse())
    assertEquals(1.0, actual.w.inRadians)
    assertEquals(0.0, actual.x.value)
    assertEquals(0.0, actual.y.value)
    assertEquals(0.0, actual.z.value)
  }

  @Test
  fun testInverse() {
    val q = Quaternion(0.75.radians, 0.3, 0.4, 0.5)
    val inv = q.inverse()
    assertEquals(q.w.inRadians, inv.w.inRadians)
    assertEquals(-q.x.value, inv.x.value)
    assertEquals(-q.y.value, inv.y.value)
    assertEquals(-q.z.value, inv.z.value)
  }
}
