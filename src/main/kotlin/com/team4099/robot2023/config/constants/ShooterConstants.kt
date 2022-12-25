package com.team4099.robot2023.config.constants

import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.rotations
import com.team4099.lib.units.perMinute

object ShooterConstants {
  // TODO change PID, ks, and kv values
  const val SHOOTER_FLYWHEEL_KP = 0.15
  const val SHOOTER_FLYWHEEL_KI = 0.0
  const val SHOOTER_FLYWHEEL_KD = 7.0

  const val SHOOTER_BACKWHEELS_KP = 0.15
  const val SHOOTER_BACKWHEELS_KI = 0.0
  const val SHOOTER_BACKWHEELS_KD = 7.0

  val SHOOTER_FLYWHEEL_KS_VOLTS = 0.4
  val SHOOTER_FLYWHEEL_KV_VOLTS_PER_RADIAN_PER_SECOND = 0.018

  val SHOOTER_BACKWHEELS_KS_VOLTS = 0.4
  val SHOOTER_BACKWHEELS_KV_VOLTS_PER_RADIAN_PER_SECOND = 0.02125

  const val SHOOTER_SENSOR_CPR = 2048
  const val SHOOTER_SENSOR_GEAR_RATIO = 1.0

  const val SUPPLY_CURRENT_LIMIT = 60.0

  const val FILTER_SIZE = 10

  //  val FOLLOWER_STATUS_FRAME_PERIOD = 255.milli.seconds

  // Target values
  val TARGET_VELOCITY_THRESHOLD = 100.rotations.perMinute

  // Physical measurements
  val ANGLE = 80.0.degrees
  val SHOOTER_HEIGHT = 33.0.inches
  val FLYWHEEL_RADIUS = 2.0.inches

  enum class ShooterState(val targetVelocity: Pair<AngularVelocity, AngularVelocity>) {
    // Pair<flywheel velocity, backwheels velocity>
    OFF(Pair(0.0.rotations.perMinute, 0.0.rotations.perMinute)),
    IDLE(
      Pair(
        500.0.rotations.perMinute,
        500.0.rotations.perMinute
      )
    ), // TODO: Fix with a better idle value
    // 1200,-3600 gave a lot more arc
    SPIN_UP_UPPER(Pair(2000.0.rotations.perMinute, -2100.0.rotations.perMinute)),
    SPIN_UP_LOWER(Pair(1100.0.rotations.perMinute, -1100.0.rotations.perMinute)),
    SHOOTER_UNJAM(Pair(-2000.0.rotations.perMinute, -2000.0.rotations.perMinute))
  }
}
