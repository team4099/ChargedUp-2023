package com.team4099.robot2023.util

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

/**
 * Multiplies a twist by a scaling factor
 *
 * @param twist The twist to multiply
 * @param factor The scaling factor for the twist components
 * @return The new twist
 */
fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d {
  return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
}

fun Pose2d.toPose3d(): Pose3d {
  return Pose3d(this.x, this.y, 0.0.meters, Rotation3d(0.0.degrees, 0.0.degrees, this.rotation))
}

fun Angle.rotateBy(angle: Angle): Angle {
  return this.inRotation2ds.rotateBy(angle.inRotation2ds).angle
}
