package com.team4099.geometry

class Pose2dTest {
  private val kEpsilon = 1E-9
  /*
   @Test
   fun testTransformBy() {
     val initial = Pose2d(Translation2d(1.0.meters, 2.0.meters), Rotation2d(45.0.degrees))
     val transformation = Transform2d(Translation2d(5.0.meters, 0.0.meters), Rotation2d(5.0.degrees))
     val transforme = initial.plus(transformation)
     assertAll(
       { assertEquals(1.0 + 5.0 / Math.sqrt(2.0), transformed.getX(), kEpsilon) },
       { assertEquals(2.0 + 5.0 / Math.sqrt(2.0), transformed.getY(), kEpsilon) }
     ) { assertEquals(50.0, transformed.getRotation().getDegrees(), kEpsilon) }
   }

   @Test
   fun testRelativeTo() {
     val initial = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0))
     val last = Pose2d(5.0, 5.0, Rotation2d.fromDegrees(45.0))
     val finalRelativeToInitial: Unit = last.relativeTo(initial)
     assertAll(
       { assertEquals(5.0 * Math.sqrt(2.0), finalRelativeToInitial.getX(), kEpsilon) },
       { assertEquals(0.0, finalRelativeToInitial.getY(), kEpsilon) }
     ) { assertEquals(0.0, finalRelativeToInitial.getRotation().getDegrees(), kEpsilon) }
   }

   @Test
   fun testEquality() {
     val one = Pose2d(0.0, 5.0, Rotation2d.fromDegrees(43.0))
     val two = Pose2d(0.0, 5.0, Rotation2d.fromDegrees(43.0))
     assertEquals(one, two)
   }

   @Test
   fun testInequality() {
     val one = Pose2d(0.0, 5.0, Rotation2d.fromDegrees(43.0))
     val two = Pose2d(0.0, 1.524, Rotation2d.fromDegrees(43.0))
     assertNotEquals(one, two)
   }

   @Test
   fun testMinus() {
     val initial = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0))
     val last = Pose2d(5.0, 5.0, Rotation2d.fromDegrees(45.0))
     val transform: Unit = last.minus(initial)
     assertAll(
       { assertEquals(5.0 * Math.sqrt(2.0), transform.getX(), kEpsilon) },
       { assertEquals(0.0, transform.getY(), kEpsilon) }
     ) { assertEquals(0.0, transform.getRotation().getDegrees(), kEpsilon) }
   }

  */
}
