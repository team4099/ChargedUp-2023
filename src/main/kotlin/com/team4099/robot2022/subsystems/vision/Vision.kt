package com.team4099.robot2022.subsystems.vision

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Vision(val io: VisionIO) : SubsystemBase() {
  val inputs = VisionIO.VisionIOInputs()

  override fun periodic() {
    Logger.getInstance().processInputs("Vision", inputs)
    io.updateInputs(inputs)
  }
}
