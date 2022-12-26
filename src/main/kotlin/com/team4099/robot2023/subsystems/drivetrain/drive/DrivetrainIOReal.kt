package com.team4099.robot2023.subsystems.drivetrain.drive

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.Constants.Universal.CANIVORE_NAME
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModuleIOReal
import edu.wpi.first.wpilibj.AnalogPotentiometer
import java.lang.Math.PI

object DrivetrainIOReal : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.FRONT_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.FRONT_LEFT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.FRONT_LEFT_MODULE_ZERO,
          "Front Left Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.FRONT_RIGHT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.FRONT_RIGHT_MODULE_ZERO,
          "Front Right Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.BACK_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.BACK_LEFT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.BACK_LEFT_MODULE_ZERO,
          "Back Left Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.BACK_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.BACK_RIGHT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.BACK_RIGHT_MODULE_ZERO,
          "Back Right Wheel"
        )
      )
    )
  }
}
