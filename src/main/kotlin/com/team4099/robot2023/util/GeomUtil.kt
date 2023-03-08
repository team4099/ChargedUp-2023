package com.team4099.robot2023.util

import org.team4099.lib.geometry.Twist2d

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
