package com.team4099.robot2022.subsystems.drivetrain

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
