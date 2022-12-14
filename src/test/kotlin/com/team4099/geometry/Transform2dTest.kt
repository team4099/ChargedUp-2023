package com.team4099.geometry

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.geometry.Transform2d
import com.team4099.lib.geometry.Translation2d
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.degrees
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class Transform2dTest {
  @Test
  fun testInverse() {
    val initial = Pose2d(Translation2d(1.0.meters, 2.0.meters), Rotation2d(45.0.degrees))
    val transform = Transform2d(Translation2d(5.0.meters, 0.0.meters), Rotation2d(5.0.degrees))
    val transformed = initial.plus(transform)
    val untransformed = transformed.plus(transform.inverse())
    assertEquals(initial, untransformed)
  }

  @Test
  fun testComposition() {
    val initial = Pose2d(Translation2d(1.0.meters, 2.0.meters), Rotation2d(45.0.degrees))
    val transform1 = Transform2d(Translation2d(5.0.meters, 0.0.meters), Rotation2d(5.0.degrees))
    val transform2 = Transform2d(Translation2d(0.0.meters, 2.0.meters), Rotation2d(5.0.degrees))
    val transformedSeparate = initial.plus(transform1).plus(transform2)
    val transformedCombined = initial.plus(transform1.plus(transform2))
    assertEquals(transformedSeparate, transformedCombined)
  }
}
