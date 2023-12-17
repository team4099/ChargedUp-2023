package com.team4099.robot2023.subsystems.drivetrain.drive

import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.Constants.Universal.CANIVORE_NAME
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModuleIOFalcon
import edu.wpi.first.wpilibj.AnalogInput

object DrivetrainIOReal : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        SwerveModuleIOFalcon(
          TalonFX(Constants.Drivetrain.FRONT_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.FRONT_LEFT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.FRONT_LEFT_MODULE_ZERO,
          Constants.Drivetrain.FRONT_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIOFalcon(
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.FRONT_RIGHT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.FRONT_RIGHT_MODULE_ZERO,
          Constants.Drivetrain.FRONT_RIGHT_MODULE_NAME
        )
      ),
      SwerveModule(
        SwerveModuleIOFalcon(
          TalonFX(Constants.Drivetrain.BACK_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.BACK_LEFT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.BACK_LEFT_MODULE_ZERO,
          Constants.Drivetrain.BACK_LEFT_MODULE_NAME
        )
      ),
      SwerveModule(
        //        object: SwerveModuleIO {
        //          override val label: String = Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        //        }
        SwerveModuleIOFalcon(
          TalonFX(Constants.Drivetrain.BACK_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogInput(Constants.Drivetrain.BACK_RIGHT_ANALOG_POTENTIOMETER),
          DrivetrainConstants.BACK_RIGHT_MODULE_ZERO,
          Constants.Drivetrain.BACK_RIGHT_MODULE_NAME
        )
      )
    )
  }
}
