package com.team4099.robot2022.subsystems.drivetrain

object DrivetrainIOSim : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(SwerveModuleIOSim("Front Left Wheel")),
      SwerveModule(SwerveModuleIOSim("Front Right Wheel")),
      SwerveModule(SwerveModuleIOSim("Back Left Wheel")),
      SwerveModule(SwerveModuleIOSim("Back Right Wheel"))
    )
  }
}
