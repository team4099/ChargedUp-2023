package com.team4099.robot2023.subsystems.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(val io: IntakeIO) : SubsystemBase() {
  val inputs = IntakeIO.IntakeIOInputs()

  init {}

  override fun periodic() {
    io.updateInputs(inputs)
  }
}
