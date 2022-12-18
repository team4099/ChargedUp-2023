package com.team4099.geometry

import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Rotation3d
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Test

internal class Rotation3dTest {
  private val kEpsilon = 1E-9

  @Test
  fun testInitAxisAngleAndRollPitchYaw() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val rot1 = Rotation3d(xAxis, (Math.PI / 3).radians)
    val rot2 = Rotation3d((Math.PI / 3).radians, 0.0.radians, 0.0.radians)
    assertEquals(rot1, rot2)
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val rot3 = Rotation3d(yAxis, (Math.PI / 3).radians)
    val rot4 = Rotation3d(0.0.radians, (Math.PI / 3).radians, 0.0.radians)
    assertEquals(rot3, rot4)
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot5 = Rotation3d(zAxis, (Math.PI / 3).radians)
    val rot6 = Rotation3d(0.0.radians, 0.0.radians, (Math.PI / 3).radians)
    assertEquals(rot5, rot6)
  }

  @Test
  fun testInitRotationMatrix() {
    // No rotation
    val R1 = Matrix.eye(Nat.N3())
    val rot1 = Rotation3d(R1)
    assertEquals(Rotation3d(), rot1)

    // 90 degree CCW rotation around z-axis
    val R2 = Matrix(Nat.N3(), Nat.N3())
    R2.assignBlock(0, 0, VecBuilder.fill(0.0, 1.0, 0.0))
    R2.assignBlock(0, 1, VecBuilder.fill(-1.0, 0.0, 0.0))
    R2.assignBlock(0, 2, VecBuilder.fill(0.0, 0.0, 1.0))
    val rot2 = Rotation3d(R2)
    val expected2 = Rotation3d(0.0.radians, 0.0.radians, 90.0.degrees)
    assertEquals(expected2, rot2)

    // Matrix that isn't orthogonal
    val R3 = MatBuilder(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    assertThrows(IllegalArgumentException::class.java) { Rotation3d(R3) }

    // Matrix that's orthogonal but not special orthogonal
    val R4 = Matrix.eye(Nat.N3()).times(2.0)
    assertThrows(IllegalArgumentException::class.java) { Rotation3d(R4) }
  }

  @Test
  fun testInitTwoVector() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    // 90 degree CW rotation around y-axis
    val rot1 = Rotation3d(xAxis, zAxis)
    val expected1 = Rotation3d(yAxis, (-Math.PI / 2.0).radians)
    assertEquals(expected1, rot1)

    // 45 degree CCW rotation around z-axis
    val rot2 = Rotation3d(xAxis, VecBuilder.fill(1.0, 1.0, 0.0))
    val expected2 = Rotation3d(zAxis, (Math.PI / 4.0).radians)
    assertEquals(expected2, rot2)

    // 0 degree rotation of x-axes
    val rot3 = Rotation3d(xAxis, xAxis)
    assertEquals(Rotation3d(), rot3)

    // 0 degree rotation of y-axes
    val rot4 = Rotation3d(yAxis, yAxis)
    assertEquals(Rotation3d(), rot4)

    // 0 degree rotation of z-axes
    val rot5 = Rotation3d(zAxis, zAxis)
    assertEquals(Rotation3d(), rot5)

    // 180 degree rotation tests. For 180 degree rotations, any quaternion with
    // an orthogonal rotation axis is acceptable. The rotation axis and initial
    // vector are orthogonal if their dot product is zero.

    // 180 degree rotation of x-axes
    val rot6 = Rotation3d(xAxis, xAxis.times(-1.0))
    val q6 = rot6.m_q
    assertEquals(0.0, q6.w.value)
    assertEquals(
      0.0, q6.x.value * xAxis[0, 0] + q6.y.value * xAxis[1, 0] + q6.z.value * xAxis[2, 0]
    )

    // 180 degree rotation of y-axes
    val rot7 = Rotation3d(yAxis, yAxis.times(-1.0))
    val q7 = rot7.m_q
    assertEquals(0.0, q7.w.inRadians)
    assertEquals(
      0.0, q7.x.value * yAxis[0, 0] + q7.y.value * yAxis[1, 0] + q7.z.value * yAxis[2, 0]
    )

    // 180 degree rotation of z-axes
    val rot8 = Rotation3d(zAxis, zAxis.times(-1.0))
    val q8 = rot8.m_q
    assertEquals(0.0, q8.w.value)
    assertEquals(
      0.0, q8.x.value * zAxis[0, 0] + q8.y.value * zAxis[1, 0] + q8.z.value * zAxis[2, 0]
    )
  }

  @Test
  fun testRadiansToDegrees() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot1 = Rotation3d(zAxis, (Math.PI / 3).radians)
    assertAll(
      { assertEquals(0.0, rot1.x.inRadians, kEpsilon) },
      { assertEquals(0.0, rot1.y.inRadians, kEpsilon) },
      { assertEquals(60.degrees.inRadians, rot1.z.inRadians, kEpsilon) }
    )
    val rot2 = Rotation3d(zAxis, (Math.PI / 4).radians)
    assertAll(
      { assertEquals(0.0, rot2.x.inRadians, kEpsilon) },
      { assertEquals(0.0, rot2.y.inRadians, kEpsilon) },
      { assertEquals(45.degrees.inRadians, rot2.z.inRadians, kEpsilon) }
    )
  }

  @Test
  fun testRadiansAndDegrees() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot1 = Rotation3d(zAxis, 45.degrees)
    assertAll(
      { assertEquals(0.0, rot1.x.inRadians, kEpsilon) },
      { assertEquals(0.0, rot1.y.inRadians, kEpsilon) },
      { assertEquals(Math.PI / 4.0, rot1.z.inRadians, kEpsilon) }
    )
    val rot2 = Rotation3d(zAxis, 30.0.degrees)
    assertAll(
      { assertEquals(0.0, rot2.x.inRadians, kEpsilon) },
      { assertEquals(0.0, rot2.y.inRadians, kEpsilon) },
      { assertEquals(Math.PI / 6.0, rot2.z.inRadians, kEpsilon) }
    )
  }

  @Test
  fun testRotationLoop() {
    var rot = Rotation3d()
    rot = rot.plus(Rotation3d(90.degrees, 0.0.radians, 0.0.degrees))
    var expected: Rotation3d? = Rotation3d(90.degrees, 0.0.radians, 0.0.radians)
    assertEquals(expected, rot)
    rot = rot.plus(Rotation3d(0.0.radians, 90.0.degrees, 0.0.degrees))
    expected =
      Rotation3d(
        VecBuilder.fill(1.0 / Math.sqrt(3.0), 1.0 / Math.sqrt(3.0), -1.0 / Math.sqrt(3.0)),
        120.0.degrees
      )
    assertEquals(expected, rot)
    rot = rot.plus(Rotation3d(0.0.radians, 0.0.radians, 90.degrees))
    expected = Rotation3d(0.0.radians, 90.degrees, 0.0.radians)
    assertEquals(expected, rot)
    rot = rot.plus(Rotation3d(0.0.radians, (-90.0).degrees, 0.0.radians))
    assertEquals(Rotation3d(), rot)
  }

  @Test
  fun testRotateByFromZeroX() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val zero = Rotation3d()
    val rotated = zero.rotateBy(Rotation3d(xAxis, 90.0.degrees))
    val expected = Rotation3d(xAxis, 90.0.degrees)
    assertEquals(expected, rotated)
  }

  @Test
  fun testRotateByFromZeroY() {
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val zero = Rotation3d()
    val rotated = zero.rotateBy(Rotation3d(yAxis, 90.degrees))
    val expected = Rotation3d(yAxis, 90.degrees)
    assertEquals(expected, rotated)
  }

  @Test
  fun testRotateByFromZeroZ() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val zero = Rotation3d()
    val rotated = zero.rotateBy(Rotation3d(zAxis, 90.0.degrees))
    val expected = Rotation3d(zAxis, 90.0.degrees)
    assertEquals(expected, rotated)
  }

  @Test
  fun testRotateByNonZeroX() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    var rot = Rotation3d(xAxis, 90.degrees)
    rot = rot.plus(Rotation3d(xAxis, 30.degrees))
    val expected = Rotation3d(xAxis, 120.degrees)
    assertEquals(expected, rot)
  }

  @Test
  fun testRotateByNonZeroY() {
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    var rot = Rotation3d(yAxis, 90.degrees)
    rot = rot.plus(Rotation3d(yAxis, 30.degrees))
    val expected = Rotation3d(yAxis, 120.degrees)
    assertEquals(expected, rot)
  }

  @Test
  fun testRotateByNonZeroZ() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    var rot = Rotation3d(zAxis, 90.degrees)
    rot = rot.plus(Rotation3d(zAxis, 30.degrees))
    val expected = Rotation3d(zAxis, 120.degrees)
    assertEquals(expected, rot)
  }

  @Test
  fun testMinus() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot1 = Rotation3d(zAxis, 70.degrees)
    val rot2 = Rotation3d(zAxis, 30.degrees)
    assertEquals(rot1.minus(rot2).z.inRadians, 40.0.degrees.inRadians, kEpsilon)
  }

  @Test
  fun testMultiplication() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot = Rotation3d(zAxis, 15.degrees)
    assertEquals(rot.times(2.0).z.inRadians, 30.degrees.inRadians, kEpsilon)
  }

  @Test
  fun testDivide() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot = Rotation3d(zAxis, 15.degrees)
    assertEquals((rot / 5.0).z.inRadians, 3.degrees.inRadians, kEpsilon)
  }

  @Test
  fun testEquality() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    var rot1 = Rotation3d(zAxis, 43.0.degrees)
    var rot2 = Rotation3d(zAxis, 43.0.degrees)
    assertEquals(rot1, rot2)
    rot1 = Rotation3d(zAxis, (-180.0).degrees)
    rot2 = Rotation3d(zAxis, (180.0).degrees)
    assertEquals(rot1, rot2)
  }

  @Test
  fun testAxisAngle() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot1 = Rotation3d(xAxis, 90.degrees)
    assertEquals(xAxis, rot1.rotation3d.axis)
    assertEquals(Math.PI / 2.0, rot1.theta.inRadians, 1e-9)
    val rot2 = Rotation3d(yAxis, 45.degrees)
    assertEquals(yAxis, rot2.rotation3d.axis)
    assertEquals(Math.PI / 4.0, rot2.theta.inRadians, 1e-9)
    val rot3 = Rotation3d(zAxis, 60.degrees)
    assertEquals(zAxis, rot3.rotation3d.axis)
    assertEquals(Math.PI / 3.0, rot3.theta.inRadians, 1e-9)
  }

  @Test
  fun testToRotation2d() {
    val rotation = Rotation3d(20.0.degrees, 30.0.degrees, 40.0.degrees)
    val expected = Rotation2d(40.degrees)
    assertEquals(expected, rotation.toRotation2d())
  }

  @Test
  fun testInequality() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val rot1 = Rotation3d(zAxis, 43.0.degrees)
    val rot2 = Rotation3d(zAxis, 43.5.degrees)
    assertNotEquals(rot1, rot2)
  }

  @Test
  fun testEmptyAxis() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 0.0)
    val rot = Rotation3d()
    assertEquals(rot.rotation3d.axis, zAxis)
  }
  /*
  @Test
  fun testInterpolate() {
    val xAxis = VecBuilder.fill(1.0, 0.0, 0.0)
    val yAxis = VecBuilder.fill(0.0, 1.0, 0.0)
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    // 50 + (70 - 50) * 0.5 = 60
    var rot1 = Rotation3d(xAxis, 50.degrees)
    var rot2 = Rotation3d(xAxis, 70.degrees)
    var interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(60.0), interpolated.getX(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getZ(), kEpsilon)

    // -160 minus half distance between 170 and -160 (15) = -175
    rot1 = Rotation3d(xAxis, Units.degreesToRadians(170))
    rot2 = Rotation3d(xAxis, Units.degreesToRadians(-160))
    interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(-175.0), interpolated.getX())
    assertEquals(Units.degreesToRadians(0.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getZ())

    // 50 + (70 - 50) * 0.5 = 60
    rot1 = Rotation3d(yAxis, Units.degreesToRadians(50))
    rot2 = Rotation3d(yAxis, Units.degreesToRadians(70))
    interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getX(), kEpsilon)
    assertEquals(Units.degreesToRadians(60.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getZ(), kEpsilon)

    // -160 minus half distance between 170 and -160 (165) = 5
    rot1 = Rotation3d(yAxis, Units.degreesToRadians(170))
    rot2 = Rotation3d(yAxis, Units.degreesToRadians(-160))
    interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(180.0), interpolated.getX(), kEpsilon)
    assertEquals(Units.degreesToRadians(-5.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(180.0), interpolated.getZ(), kEpsilon)

    // 50 + (70 - 50) * 0.5 = 60
    rot1 = Rotation3d(zAxis, Units.degreesToRadians(50))
    rot2 = Rotation3d(zAxis, Units.degreesToRadians(70))
    interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getX(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(60.0), interpolated.getZ(), kEpsilon)

    // -160 minus half distance between 170 and -160 (15) = -175
    rot1 = Rotation3d(zAxis, Units.degreesToRadians(170))
    rot2 = Rotation3d(zAxis, Units.degreesToRadians(-160))
    interpolated = rot1.interpolate(rot2, 0.5)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getX(), kEpsilon)
    assertEquals(Units.degreesToRadians(0.0), interpolated.getY(), kEpsilon)
    assertEquals(Units.degreesToRadians(-175.0), interpolated.getZ(), kEpsilon)
  }

   */
}
