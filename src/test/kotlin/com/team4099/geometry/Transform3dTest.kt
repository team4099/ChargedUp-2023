package com.team4099.geometry

import com.team4099.lib.geometry.Pose3d
import com.team4099.lib.geometry.Rotation3d
import com.team4099.lib.geometry.Transform3d
import com.team4099.lib.geometry.Translation3d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import edu.wpi.first.math.VecBuilder
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class Transform3dTest {
  @Test
  fun testInverse() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val initial =
      Pose3d(Translation3d(1.0.meters, 2.0.meters, 3.0.meters), Rotation3d(zAxis, 45.0.degrees))
    val transform =
      Transform3d(
        Translation3d(5.0.meters, 4.0.meters, 3.0.meters), Rotation3d(zAxis, 5.0.degrees)
      )
    val transformed = initial.plus(transform)
    val untransformed = transformed.plus(transform.inverse())
    assertEquals(initial, untransformed)
  }

  @Test
  fun testComposition() {
    val zAxis = VecBuilder.fill(0.0, 0.0, 1.0)
    val initial =
      Pose3d(Translation3d(1.0.meters, 2.0.meters, 3.0.meters), Rotation3d(zAxis, 45.0.degrees))
    val transform1 =
      Transform3d(
        Translation3d(5.0.meters, 0.0.meters, 0.0.meters), Rotation3d(zAxis, 5.0.degrees)
      )
    val transform2 =
      Transform3d(
        Translation3d(0.0.meters, 2.0.meters, 0.0.meters), Rotation3d(zAxis, 5.0.degrees)
      )
    val transformedSeparate = initial.plus(transform1).plus(transform2)
    val transformedCombined = initial.plus(transform1.plus(transform2))
    assertEquals(transformedSeparate, transformedCombined)
  }
}
