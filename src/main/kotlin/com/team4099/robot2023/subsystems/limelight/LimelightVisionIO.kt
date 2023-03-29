package com.team4099.robot2023.subsystems.limelight

import com.team4099.robot2023.util.LimelightReading
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

interface LimelightVisionIO {

  class LimelightVisionIOInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var fps = 0.0
    var validReading = false
    var angle = 0.degrees
    var retroTargets = listOf<LimelightReading>()

    override fun fromLog(table: LogTable?) {
      table?.getDouble("timestampSeconds", timestamp.inSeconds)?.let { timestamp = it.seconds }
      table?.getDouble("fps", fps)?.let { fps = it }
      table?.getBoolean("validReading", validReading)?.let { validReading = it }
      table?.getDouble("simpleAngleDegrees", angle.inDegrees)?.let { angle = it.degrees }
      val numOfTargets = table?.getInteger("numOfTargets", 0) ?: 0
      val retrievedTargets = mutableListOf<LimelightReading>()
      for (targetIndex in 0 until numOfTargets) {
        val targetTx: Angle? = table?.getDouble("Detection/$targetIndex/tx", 0.0)?.degrees
        val targetTy: Angle? = table?.getDouble("Detection/$targetIndex/tx", 0.0)?.degrees
        val targetTxPixels: Double? = table?.getDouble("Detection/$targetIndex/txPixels", 0.0)
        val targetTyPixels: Double? = table?.getDouble("Detection/$targetIndex/tyPixels", 0.0)
        val targetTs: Angle? = table?.getDouble("Detection/$targetIndex/tyPixels", 0.0)?.degrees
        if (targetTx != null &&
          targetTy != null &&
          targetTxPixels != null &&
          targetTyPixels != null &&
          targetTs != null
        ) {
          retrievedTargets.add(
            LimelightReading(targetTx, targetTy, targetTxPixels, targetTyPixels, targetTs)
          )
        }
      }
      retroTargets = retrievedTargets.toList()
    }

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("fps", fps)
      table?.put("validReading", validReading)
      table?.put("simpleAngleDegrees", angle.inDegrees)
      table?.put("numOfTargets", retroTargets.size.toLong())
      table?.put("cornersX", retroTargets.map { it.txPixel }.toDoubleArray())
      table?.put("cornersY", retroTargets.map { it.tyPixel }.toDoubleArray())
      for (i in retroTargets.indices) {
        table?.put("Detection/$i/txDegrees", retroTargets[i].tx.inDegrees)
        table?.put("Detection/$i/tyDegrees", retroTargets[i].ty.inDegrees)
        table?.put("Detection/$i/tyPixels", retroTargets[i].tyPixel)
        table?.put("Detection/$i/txPixels", retroTargets[i].txPixel)
        table?.put("Detection/$i/tsDegrees", retroTargets[i].ts.inDegrees)
      }
    }
  }

  fun updateInputs(inputs: LimelightVisionIOInputs) {}

  fun setPipeline(pipelineIndex: Int) {}

  fun setLeds(enabled: Boolean) {}
}
