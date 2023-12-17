package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.VisionConstants.Limelight.LIMELIGHT_NAME
import com.team4099.robot2023.util.LimelightReading
import com.team4099.utils.LimelightHelpers
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.milli

object LimelightVisionIOReal : LimelightVisionIO {

  private val ledEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("ledMode")
  private val pipelineEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("pipeline")
  private val validEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tv")
  private val latencyEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tl")
  private val captureLatencyEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("cl")
  private val dataEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tcornxy")
  private val angleEntry: NetworkTableEntry =
    NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx")

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    val totalLatency =
      (
        latencyEntry.getDouble(0.0).milli.seconds +
          captureLatencyEntry.getDouble(0.0).milli.seconds
        )

    inputs.timestamp = Clock.realTimestamp - totalLatency
    inputs.angle = angleEntry.getDouble(0.0).degrees
    inputs.fps = 1000 / totalLatency.inMilliseconds
    inputs.validReading = true

    inputs.retroTargets =
      LimelightHelpers.getLatestResults(LIMELIGHT_NAME).targetingResults.targets_Retro.map {
        LimelightReading(it)
      }
  }

  override fun setPipeline(pipelineIndex: Int) {
    pipelineEntry.setInteger(pipelineIndex.toLong())
  }

  override fun setLeds(enabled: Boolean) {
    ledEntry.setBoolean(enabled)
  }
}
