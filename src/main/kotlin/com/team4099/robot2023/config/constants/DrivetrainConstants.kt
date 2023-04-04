package com.team4099.robot2023.config.constants

import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.radiansPerSecondPerRadiansPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond
import kotlin.math.sqrt

object DrivetrainConstants {
  const val MINIMIZE_SKEW = false

  const val WHEEL_COUNT = 4
  val WHEEL_DIAMETER = 3.827.inches
  val DRIVETRAIN_LENGTH = 22.750.inches
  val DRIVETRAIN_WIDTH = 22.750.inches

  val DOCKING_GYRO_SETPOINT = 0.0.degrees
  val DOCKING_GYRO_TOLERANCE = 2.5.degrees
  val DOCKING_TIME_THRESHOLD = 1.0.seconds

  var DRIVE_SETPOINT_MAX = 15.feet.perSecond
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

  val FRONT_LEFT_MODULE_ZERO = 1.33.radians
  val FRONT_RIGHT_MODULE_ZERO = 4.51.radians
  val BACK_LEFT_MODULE_ZERO = 3.20.radians
  val BACK_RIGHT_MODULE_ZERO = 0.94.radians

  val STEERING_COMPENSATION_VOLTAGE = 10.volts
  val DRIVE_COMPENSATION_VOLTAGE = 12.volts

  val DRIVE_WHEEL_INERTIA = 0.025.kilo.grams.meterSquared
  val STEERING_WHEEL_INERTIA = 0.004096955.kilo.grams.meterSquared

  object PID {
    val AUTO_POS_KP: ProportionalGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return 8.0.meters.perSecond / 1.0.meters
        } else {
          return 7.0.meters.perSecond / 1.0.meters
        }
      }
    val AUTO_POS_KI: IntegralGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
        } else {
          return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
        }
      }

    val AUTO_POS_KD: DerivativeGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return (0.05.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
        } else {
          return (0.0.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
        }
      }

    val AUTO_THETA_ALLOWED_ERROR = 3.degrees

    val AUTO_THETA_PID_KP = 10.degrees.perSecond / 1.degrees
    val AUTO_THETA_PID_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val AUTO_THETA_PID_KD =
      (0.0.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val SIM_AUTO_THETA_PID_KP = 7.degrees.perSecond / 1.degrees
    val SIM_AUTO_THETA_PID_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val SIM_AUTO_THETA_PID_KD =
      (0.5.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val AUTO_LEVEL_KP = 1.meters.perSecond / 1.0.degrees // tune this
    val AUTO_LEVEL_KI = 0.0.meters.perSecond / (1.0.degrees * 1.seconds) // tune this
    val AUTO_LEVEL_KD = 0.0.meters.perSecond / (1.0.degrees.perSecond) // tune this
    val AUTO_LEVEL_MAX_VEL_SETPOINT = 2.degrees.perSecond
    val AUTO_LEVEL_MAX_ACCEL_SETPOINT = 2.degrees.perSecond.perSecond

    val MAX_AUTO_ANGULAR_VEL = 270.0.degrees.perSecond
    val MAX_AUTO_ANGULAR_ACCEL = 600.0.degrees.perSecond.perSecond

    val STEERING_KP = 10.0.volts / 45.degrees
    val STEERING_KI = 0.0.volts.perDegreeSeconds
    val STEERING_KD = 0.0.volts.perDegreePerSecond

    val STEERING_KFF = 0.0.volts / 1.0.radians.perSecond // 0.0375

    val DRIVE_KP = 2.6829.volts / 1.meters.perSecond
    val DRIVE_KI = 0.0.volts / (1.meters.perSecond * 1.seconds)
    val DRIVE_KD = 0.0.volts / 1.meters.perSecond.perSecond

    val DRIVE_KFF = 12.0.volts / 4.1675.meters.perSecond

    val DRIVE_KS = 0.5.volts
    val DRIVE_KV = 0.0.volts / 1.0.meters.perSecond
    val DRIVE_KA = 0.0.volts / 1.0.meters.perSecond.perSecond

    //    val DRIVE_KS = 0.23677.volts
    //    val DRIVE_KV = 2.2678.volts / 1.0.meters.perSecond
    //    val DRIVE_KA = 0.40499.volts / 1.0.meters.perSecond.perSecond

    val SIM_DRIVE_KS = 0.116970.volts
    val SIM_DRIVE_KV = 2.7.volts / 1.0.meters.perSecond

    val SIM_DRIVE_KP = 1.5.volts / 1.meters.perSecond
    val SIM_DRIVE_KI = 0.0.volts / (1.meters.perSecond * 1.seconds)
    val SIM_DRIVE_KD = 0.0.volts / 1.meters.perSecond.perSecond

    val SIM_STEERING_KP = 0.4.volts.perDegree
    val SIM_STEERING_KI = 0.0.volts.perDegreeSeconds
    val SIM_STEERING_KD = 0.0.volts.perDegreePerSecond
  }
}
