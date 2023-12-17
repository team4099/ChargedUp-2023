package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModule
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.SwerveModuleIO

interface DrivetrainIO {
  fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        object : SwerveModuleIO {
          override val label = "Front Left Wheel"
        }),
      SwerveModule(
        object : SwerveModuleIO {
          override val label = "Front Right Wheel"
        }),
      SwerveModule(
        object : SwerveModuleIO {
          override val label = "Back Left Wheel"
        }),
      SwerveModule(
        object : SwerveModuleIO {
          override val label = "Back Right Wheel"
        })
    )
  }
}
