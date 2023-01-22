package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.milli

object Constants {
  object Universal {
    val SIM_MODE = Tuning.SimType.SIM

    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    val CANIVORE_NAME = "kestrel_vore"
    val LOG_FOLDER = "/media/sda1/"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1
    val USE_TIMING = true
  }

  object Tuning {

    val TUNING_MODE = false
    val SIMULATE_DRIFT = true

    enum class SimType {
      SIM,
      REPLAY
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

  object Gyro {
    const val PIGEON_2_ID = 1337
  }

  object Alert {
    val TABS = arrayOf("Pre-match", "In-match")
  }
}
