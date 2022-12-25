package com.team4099.robot2023.config.constants

import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.feet
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.perSecond
import kotlin.math.sqrt

object DrivetrainConstants {
  const val WHEEL_COUNT = 4
  val WHEEL_DIAMETER = 3.785.inches
  val DRIVETRAIN_LENGTH = 22.750.inches
  val DRIVETRAIN_WIDTH = 22.750.inches

  val DRIVE_SETPOINT_MAX = 15.feet.perSecond
  val TURN_SETPOINT_MAX =
    (DRIVE_SETPOINT_MAX.inMetersPerSecond / DRIVETRAIN_LENGTH.inMeters / 2 * sqrt(2.0))
      .radians
      .perSecond // 648

  // cruise velocity and accel for steering motor
  val STEERING_VEL_MAX = 1393.2.degrees.perSecond
  val STEERING_ACCEL_MAX = 13932.degrees.perSecond.perSecond

  const val GYRO_RATE_COEFFICIENT = 0.0 // TODO: Change this value

  val SLOW_AUTO_VEL = 2.meters.perSecond
  val SLOW_AUTO_ACCEL = 2.0.meters.perSecond.perSecond

  val MAX_AUTO_VEL = 3.meters.perSecond // 4
  val MAX_AUTO_ACCEL = 3.meters.perSecond.perSecond // 3

  val MAX_AUTO_BRAKE_VEL = 0.5.meters.perSecond // 4
  val MAX_AUTO_BRAKE_ACCEL = 0.5.meters.perSecond.perSecond // 3

  const val DRIVE_SENSOR_CPR = 2048
  const val STEERING_SENSOR_CPR = 2048

  const val DRIVE_SENSOR_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
  const val STEERING_SENSOR_GEAR_RATIO = 7.0 / 150.0

  val ALLOWED_STEERING_ANGLE_ERROR = 1.degrees

  val STEERING_SUPPLY_CURRENT_LIMIT = 20.0.amps
  val DRIVE_SUPPLY_CURRENT_LIMIT = 35.0.amps
  val DRIVE_THRESHOLD_CURRENT_LIMIT = 60.0.amps
  val DRIVE_TRIGGER_THRESHOLD_TIME = 0.1.seconds

  val DRIVE_STATOR_CURRENT_LIMIT = 65.0.amps
  val DRIVE_STATOR_THRESHOLD_CURRENT_LIMIT = 80.0.amps
  val DRIVE_STATOR_TRIGGER_THRESHOLD_TIME = 1.0.seconds

  val FRONT_LEFT_MODULE_ZERO = 2.687684.radians
  val FRONT_RIGHT_MODULE_ZERO = 3.274.radians
  val BACK_RIGHT_MODULE_ZERO = 4.608515.radians
  val BACK_LEFT_MODULE_ZERO = 5.204072.radians

  object PID {
    const val AUTO_POS_KP = 2.0
    const val AUTO_POS_KI = 0.0
    const val AUTO_POS_KD = 0.75

    const val AUTO_THETA_PID_KP = 7.6
    const val AUTO_THETA_PID_KI = 0.0
    const val AUTO_THETA_PID_KD = 0.0

    val MAX_AUTO_ANGULAR_VEL = 270.0.degrees.perSecond
    val MAX_AUTO_ANGULAR_ACCEL = 600.0.degrees.perSecond.perSecond

    const val STEERING_KP = 0.125
    const val STEERING_KI = 0.0
    const val STEERING_KD = 0.0

    const val STEERING_KFF = 0.0 // 0.0375

    const val DRIVE_KP = 0.050046
    const val DRIVE_KI = 0.0
    const val DRIVE_KD = 0.0
    const val DRIVE_KFF = 0.0

    val DRIVE_KS = 0.23677.volts
    val DRIVE_KV = 2.2678.volts / 1.0.meters.perSecond
    val DRIVE_KA = 0.40499.volts / 1.0.meters.perSecond.perSecond

    val SIM_DRIVE_KS = 0.116970.volts
    val SIM_DRIVE_KV = 0.133240.volts / 1.0.meters.perSecond

    val SIM_DRIVE_KP = 0.9
    val SIM_DRIVE_KI = 0.0
    val SIM_DRIVE_KD = 0.0

    val SIM_STEERING_KP = 23.0
    val SIM_STEERING_KI = 0.0
    val SIM_STEERING_KD = 0.0
  }
}
