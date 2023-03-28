package com.team4099.robot2023.subsystems.limelight

import com.team4099.robot2023.util.LimelightHelpers
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees

interface LimelightVisionIO {

  class LimelightVisionIOInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var fps = 0.0
    var validReading = false
    var angle = 0.degrees
    var corners = listOf<Pair<Double, Double>>()
    var retroTargets = listOf<LimelightHelpers.LimelightTarget_Retro>()

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  fun updateInputs(inputs: LimelightVisionIOInputs) {}

  fun setPipeline(pipelineIndex: Int) {}

  fun setLeds(enabled: Boolean) {}
}
