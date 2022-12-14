package com.team4099.robot2022.subsystems.drivetrain

interface GyroIO {
  open class GyroIOInputs {
    var gyroYaw = 0.0
    var gyroPitch = 0.0
    var gyroRoll = 0.0
    var gyroVelocity = 0.0
    var gyroName = "navx2"
    var gyroConnected = false
  }
}
