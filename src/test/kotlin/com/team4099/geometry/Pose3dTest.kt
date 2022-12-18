package com.team4099.geometry

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Pose3d
import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Rotation3d
import com.team4099.lib.geometry.Transform3d
import com.team4099.lib.geometry.Translation3d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.VecBuilder
import org.junit.jupiter.api.Assertions.assertAll
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Test

class Pose3dTest {
  private val kEpsilon = 1E-9

  @Test
  fun testInit() {
    val pose2d = Pose2d(0.0.meters, 0.0.meters, Rotation2d(45.degrees))
    val translation3d = Translation3d(pose2d.x, pose2d.y, 0.0.meters)
    val rotation3d = Rotation3d(0.0.radians, 0.0.radians, pose2d.rotation.theta)
    assertEquals(Pose3d(pose2d), Pose3d(translation3d, rotation3d))
  }

  @Test
  fun testTransformByRotations() {
    val initialPose =
      Pose3d(
        Translation3d(0.0.meters, 0.0.meters, 0.0.meters),
        Rotation3d(0.0.degrees, 0.0.degrees, 0.0.degrees)
      )

    val transform1 =
      Transform3d(
        Translation3d(0.0.meters, 0.0.meters, 0.0.meters),
        Rotation3d(90.0.degrees, 45.0.degrees, 0.0.degrees)
      )

    val transform2 =
      Transform3d(
        Translation3d(0.0.meters, 0.0.meters, 0.0.meters),
        Rotation3d((-90.0).degrees, 0.0.degrees, 0.0.degrees)
      )

    val transform3 =
      Transform3d(
        Translation3d(0.0.meters, 0.0.meters, 0.0.meters),
        Rotation3d(0.0.degrees, (-45.0).degrees, 0.0.degrees)
      )

    // This sequence of rotations should diverge from the origin and eventually return to it. When
    // each rotation occurs, it should be performed intrinsicly, i.e. 'from the viewpoint' of and
    // with
    // the axes of the pose that is being transformed, just like how the translation is done 'from
    // the
    // viewpoint' of the pose that is being transformed.
    val finalPose =
      initialPose.transformBy(transform1).transformBy(transform2).transformBy(transform3)

    assertAll(
      {
        assertEquals(
          finalPose.m_rotation.x.inDegrees, initialPose.m_rotation.x.inDegrees, kEpsilon
        )
      },
      {
        assertEquals(
          finalPose.m_rotation.y.inDegrees, initialPose.m_rotation.y.inDegrees, kEpsilon
        )
      },
      {
        assertEquals(
          finalPose.m_rotation.z.inDegrees, initialPose.m_rotation.z.inDegrees, kEpsilon
        )
      }
    )
  }

  @Test
  fun testTransformBy() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    val initial =
      Pose3d(Translation3d(1.0.meters, 2.0.meters, 0.0.meters), Rotation3d(zAxis, 45.0.degrees))
    val transformation =
      Transform3d(Translation3d(5.0.meters, 0.0.meters, 0.0.meters), Rotation3d(zAxis, 5.degrees))

    val transformed = initial.plus(transformation)

    assertAll(
      { assertEquals(1.0 + 5.0 / Math.sqrt(2.0), transformed.m_translation.x.value, kEpsilon) },
      { assertEquals(2.0 + 5.0 / Math.sqrt(2.0), transformed.m_translation.y.value, kEpsilon) },
      { assertEquals(50.0, transformed.m_rotation.z.inDegrees, kEpsilon) }
    )
  }

  @Test
  fun testRelativeTo() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    val initial = Pose3d(0.0.meters, 0.0.meters, 0.0.meters, Rotation3d(zAxis, 45.0.degrees))
    val last = Pose3d(5.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 45.0.degrees))

    val finalRelativeToInitial = last.relativeTo(initial)

    assertAll(
      {
        assertEquals(5.0 * Math.sqrt(2.0), finalRelativeToInitial.m_translation.x.value, kEpsilon)
      },
      { assertEquals(0.0, finalRelativeToInitial.m_translation.y.value, kEpsilon) },
      { assertEquals(0.0, finalRelativeToInitial.m_translation.z.value, kEpsilon) }
    )
  }

  @Test
  fun testEquality() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    val one = Pose3d(0.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 43.0.degrees))
    val two = Pose3d(0.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 43.0.degrees))
    assertEquals(one, two)
  }

  @Test
  fun testInequality() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    val one = Pose3d(0.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 43.0.degrees))
    val two = Pose3d(0.0.meters, 1.524.meters, 0.0.meters, Rotation3d(zAxis, 43.0.degrees))
    assertNotEquals(one, two)
  }

  @Test
  fun testMinus() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)

    val initial = Pose3d(0.0.meters, 0.0.meters, 0.0.meters, Rotation3d(zAxis, 45.0.degrees))
    val last = Pose3d(5.0.meters, 5.0.meters, 0.0.meters, Rotation3d(zAxis, 45.degrees))

    val transform = last.minus(initial)

    assertAll(
      { assertEquals(5.0 * Math.sqrt(2.0), transform.x.value, kEpsilon) },
      { assertEquals(0.0, transform.y.value, kEpsilon) },
      { assertEquals(0.0, transform.m_rotation.z.inDegrees, kEpsilon) }
    )
  }

  @Test
  fun testMultiplication() {
    val translation3d = Translation3d(1.0.meters, 1.0.meters, 1.0.meters)
    val rotation3d = Rotation3d(VecBuilder.fill(0.0, 0.0, 0.0), 45.0.degrees)
    val pose3d = Pose3d(translation3d, rotation3d)
    assertEquals(pose3d * 3.0, Pose3d(translation3d * 3.0, rotation3d * 3.0))
  }

  @Test
  fun testDivide() {
    val translation3d = Translation3d(1.0.meters, 1.0.meters, 1.0.meters)
    val rotation3d = Rotation3d(VecBuilder.fill(0.0, 0.0, 0.0), 45.0.degrees)
    val pose3d = Pose3d(translation3d, rotation3d)
    assertEquals(pose3d / 3.0, Pose3d(translation3d / 3.0, rotation3d / 3.0))
  }

  @Test
  fun testToPose2d() {
    val pose =
      Pose3d(
        1.0.meters,
        2.0.meters,
        3.0.meters,
        Rotation3d(20.0.degrees, 30.0.degrees, 40.0.degrees)
      )
    val expected = Pose2d(1.0.meters, 2.0.meters, Rotation2d(40.0.degrees))

    assertEquals(expected, pose.toPose2d())
  }
}
