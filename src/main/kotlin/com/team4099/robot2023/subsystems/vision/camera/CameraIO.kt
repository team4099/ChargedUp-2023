package com.team4099.robot2023.subsystems.vision.camera

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

interface CameraIO {
  class CameraInputs : LoggableInputs {
    var timestamps = listOf<Time>()
    var frames = listOf<DoubleArray>()
    var fps = 0.0

    override fun toLog(table: LogTable?) {
      table?.put("timestampsSeconds", timestamps.map { it.inSeconds }.toDoubleArray())
      table?.put("frameCount", frames.size.toDouble())
      for (i in frames.indices) {
        table?.put("Frame/$i", frames[i])
      }
      table?.put("fps", fps)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDoubleArray("timestampsSeconds", timestamps.map { it.inSeconds }.toDoubleArray())
        ?.let { returnedTimestamps -> timestamps = returnedTimestamps.map { it.seconds } }

      val frameCount = table?.getDouble("frameCount", 0.0)?.toInt() ?: 0
      val tempFrames = mutableListOf<DoubleArray>()
      for (i in 0 until frameCount) {
        tempFrames.add(table?.getDoubleArray("Frame/$i", DoubleArray(0)) ?: DoubleArray(0))
      }
      frames = tempFrames.toList()

      table?.getDouble("fps", fps)?.let { fps = it }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}
