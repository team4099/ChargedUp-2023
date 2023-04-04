package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.util.LimelightReading
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians

object LimelightVisionIOSim: LimelightVisionIO {

  private val tunableTx = LoggedTunableValue("LimelightSim/txDegrees", (8.612154).degrees, Pair({it.inDegrees}, {it.degrees}))
  private val tunableTy = LoggedTunableValue("LimelightSim/tyDegrees", (-1.6576093255536648).degrees, Pair({it.inDegrees}, {it.degrees}))

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    inputs.timestamp = Clock.realTimestamp
    inputs.angle = 0.0.radians
    inputs.fps = 90.0
    inputs.validReading = true
    inputs.retroTargets = listOf(LimelightReading(tunableTx.get(), tunableTy.get(), 0.0, 0.0, 10.degrees))
  }
}
