package com.team4099.robot2023.subsystems.limelight

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class LimelightVision(val io: LimelightVisionIO) : SubsystemBase() {
  val inputs = LimelightVisionIO.LimelightVisionIOInputs()

  override fun periodic() {
    io.updateInputs(inputs)
    Logger.getInstance().processInputs("LimelightVision", inputs)
  }
}
