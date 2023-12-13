package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.milli

typealias GamePiece = Constants.Universal.GamePiece

typealias Substation = Constants.Universal.Substation

typealias NodeTier = Constants.Universal.NodeTier

object Constants {
  object Universal {
    val SIM_MODE = Tuning.SimType.SIM
    const val REAL_FIELD = false

    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    val CANIVORE_NAME = "FalconVore"
    val LOG_FOLDER = "/media/sda1/"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1

    enum class GamePiece {
      CUBE,
      CONE,
      NONE;

      inline fun toClassName(): String {
        return when (this) {
          CUBE -> "cube"
          CONE -> "cone"
          else -> "none"
        }
      }
    }

    enum class NodeTier {
      HYBRID,
      MID,
      HIGH,
      NONE
    }

    enum class Substation {
      DOUBLE_SUBSTATION_LEFT,
      DOUBLE_SUBSTATION_RIGHT,
      SINGLE_SUBSTATION,
      NONE,
    }
  }

  object AprilTagIds {
    const val BLUE_DOUBLE_SUBSTATION_ID = 4
  }

  object Tuning {

    const val TUNING_MODE = false
    const val DEBUGING_MODE = true
    const val SIMULATE_DRIFT = false
    const val DRIFT_CONSTANT = 0.001

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
    const val FRONT_LEFT_ANALOG_POTENTIOMETER = 1

    val FRONT_LEFT_MODULE_NAME = "Front Left Wheel"

    const val FRONT_RIGHT_DRIVE_ID = 12
    const val FRONT_RIGHT_STEERING_ID = 22
    const val FRONT_RIGHT_ANALOG_POTENTIOMETER = 2

    val FRONT_RIGHT_MODULE_NAME = "Front Right Wheel"

    const val BACK_LEFT_DRIVE_ID = 13
    const val BACK_LEFT_STEERING_ID = 23
    const val BACK_LEFT_ANALOG_POTENTIOMETER = 0

    val BACK_LEFT_MODULE_NAME = "Back Left Wheel"

    const val BACK_RIGHT_DRIVE_ID = 14
    const val BACK_RIGHT_STEERING_ID = 24
    const val BACK_RIGHT_ANALOG_POTENTIOMETER = 3

    val BACK_RIGHT_MODULE_NAME = "Back Right Wheel"
  }

  // leader is right motor and follower is left
  object Elevator {
    const val LEADER_MOTOR_ID = 42
    const val FOLLOWER_MOTOR_ID = 41
  }

  object Gyro {
    const val PIGEON_2_ID = 1
  }

  object Manipulator {
    const val INTAKE_MOTOR_ID = 51
    const val ARM_MOTOR_ID = 52
  }

  object Alert {
    val TABS = arrayOf("Pre-match", "In-match")
  }

  object Intake {
    const val ROLLER_MOTOR_ID = 32
    const val ARM_MOTOR_ID = 31
    const val REV_ENCODER_PORT = 7
  }

  object Led {
    const val LED_CANDLE_ID = 61
    const val LED_BLINKEN_ID = 1
  }
}
