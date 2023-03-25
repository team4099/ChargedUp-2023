package com.team4099.robot2023.subsystems.motorchecker

object MotorChecker {

  val motors = mutableListOf<Motor<MotorType>>()

  fun add(motor: Motor<MotorType>) {
    motors.add(motor)
  }

  fun periodic() {}
}
