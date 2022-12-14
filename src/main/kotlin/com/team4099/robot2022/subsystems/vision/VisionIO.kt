package com.team4099.robot2022.subsystems.vision

import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.targeting.PhotonTrackedTarget

interface VisionIO {
  class VisionIOInputs : LoggableInputs {
    var hasTargets: Boolean = false
    var numOfTargets = 0.0
    val targets: List<PhotonTrackedTarget> = mutableListOf()
    var latency = 0.0.seconds
    var bestX = 0.0.meters
    var bestY = 0.0.meters
    var bestZ = 0.0.meters
    var bestRoll = 0.0.degrees
    var bestPitch = 0.0.degrees
    var bestYaw = 0.0.degrees

    override fun toLog(table: LogTable?) {
      table?.put("hasTargets", hasTargets)
      table?.put("latencyMS", latency.inSeconds)
      table?.put("bestX", bestX.inMeters)
      table?.put("bestY", bestY.inMeters)
      table?.put("bestZ", bestZ.inMeters)
      table?.put("bestRoll", bestRoll.inDegrees)
      table?.put("bestPitch", bestPitch.inDegrees)
      table?.put("bestYaw", bestYaw.inDegrees)
      table?.put("numOfTargets", numOfTargets)
    }

    override fun fromLog(table: LogTable?) {
      table?.getBoolean("hasTargets", hasTargets)?.let { hasTargets = it }
      table?.getDouble("latencyMS", latency.inSeconds)?.let { latency = it.seconds }
      table?.getDouble("bestX", bestX.inMeters)?.let { bestX = it.meters }
      table?.getDouble("bestY", bestY.inMeters)?.let { bestY = it.meters }
      table?.getDouble("bestZ", bestZ.inMeters)?.let { bestZ = it.meters }
      table?.getDouble("bestRoll", bestRoll.inDegrees)?.let { bestRoll = it.degrees }
      table?.getDouble("bestPitch", bestPitch.inDegrees)?.let { bestPitch = it.degrees }
      table?.getDouble("bestRoll", bestYaw.inDegrees)?.let { bestYaw = it.degrees }
      table?.getDouble("numOfTargets", numOfTargets)?.let { numOfTargets = it }
    }
  }

  fun updateInputs(inputs: VisionIO.VisionIOInputs) {}
}
