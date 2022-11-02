package com.team4099.robot2022.config.constants

import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.centi
import com.team4099.lib.units.milli
import edu.wpi.first.wpilibj.RobotBase

object Constants {
  object Universal {
    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    val CANIVORE_NAME = "kestrel_vore"

    val LOOP_PERIOD_TIME = 2.centi.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1
    val USE_TIMING = true
  }

  object Tuning {

    val TUNING_MODE = false

    enum class RobotType {
      REAL,
      SIM,
      REPLAY
    }

    val type: RobotType
      get() {
        if (RobotBase.isReal()) {
          return RobotType.REAL
        } else {
          return RobotType.REPLAY
        }
      }
  }

  object Joysticks {
    const val DRIVER_PORT = 0
    const val SHOTGUN_PORT = 1
    const val TECHNICIAN_PORT = 2

    const val THROTTLE_DEADBAND = 0.05
    const val TURN_DEADBAND = 0.05
  }

  object Drivetrain {
    const val FRONT_LEFT_DRIVE_ID = 11
    const val FRONT_LEFT_STEERING_ID = 21
    const val FRONT_LEFT_ANALOG_POTENTIOMETER = 0

    const val FRONT_RIGHT_DRIVE_ID = 15
    const val FRONT_RIGHT_STEERING_ID = 25
    const val FRONT_RIGHT_ANALOG_POTENTIOMETER = 1

    const val BACK_RIGHT_DRIVE_ID = 13
    const val BACK_RIGHT_STEERING_ID = 23
    const val BACK_RIGHT_ANALOG_POTENTIOMETER = 2

    const val BACK_LEFT_DRIVE_ID = 14
    const val BACK_LEFT_STEERING_ID = 24
    const val BACK_LEFT_ANALOG_POTENTIOMETER = 3
  }

  object Shooter {
    const val FLYWHEEL_MOTOR_ID = 51
    const val BACK_WHEELS_MOTOR_ID = 52
  }

  object Feeder {
    const val FLOOR_MOTOR_ID = 61
    const val VERTICAL_MOTOR_ID = 62

    const val TOP_DIO_PIN = 9
    const val BOTTOM_DIO_PIN = 8
  }

  object Intake {
    const val INTAKE_MOTOR = 31
    const val ARM_SOLENOID = 8
  }

  object LED {
    const val LED_CONTROLLER_ID = 1
    const val TAB = "LED"
  }

  object TelescopingClimber {
    const val SOLENOID_ID = 0
    const val R_ARM_ID = 42
    const val L_ARM_ID = 41
  }

  object PivotClimber {
    const val IN_PORT = 11
    const val OUT_PORT = 12
  }
}
