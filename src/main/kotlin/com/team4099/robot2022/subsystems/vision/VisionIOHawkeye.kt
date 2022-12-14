package com.team4099.robot2022.subsystems.vision

import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.milli
import org.photonvision.PhotonCamera

object VisionIOHawkeye : VisionIO {
  val camera: PhotonCamera = PhotonCamera("hawkeye")
  var bestXReading = 0.0.meters
  var bestYReading = 0.0.meters
  var bestZReading = 0.0.meters
  var bestRollReading = 0.0.degrees
  var bestPitchReading = 0.0.degrees
  var bestYawReading = 0.0.degrees

  var latency = 0.milli.seconds
  var hasTargets = false

  init {
    // Hawkeye doesn't work without this
    PhotonCamera.setVersionCheckEnabled(false)
  }

  override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
    val result = camera.latestResult
    var bestX = 0.meters
    var bestY = 0.meters
    var bestZ = 0.0.meters
    var bestRoll = 0.0.degrees
    var bestPitch = 0.0.degrees
    var bestYaw = 0.0.degrees
    val hasTargets = result.hasTargets()
    if (hasTargets) {
      bestX = result.bestTarget.bestCameraToTarget.x.meters
      bestY = result.bestTarget.bestCameraToTarget.y.meters
      bestZ = result.bestTarget.bestCameraToTarget.z.meters
      bestRoll = result.bestTarget.bestCameraToTarget.rotation.x.radians
      bestPitch = result.bestTarget.bestCameraToTarget.rotation.y.radians
      bestYaw = result.bestTarget.bestCameraToTarget.rotation.z.radians
    }

    println(result.latencyMillis.seconds.inSeconds)
    println(bestX.inMeters)
    println(bestY.inMeters)
    println(bestZ.inMeters)

    inputs.latency = result.latencyMillis.seconds
    inputs.bestX = bestX
    inputs.bestY = bestY
    inputs.bestZ = bestZ
    inputs.bestRoll = bestRoll
    inputs.bestPitch = bestPitch
    inputs.bestYaw = bestYaw
    inputs.hasTargets = hasTargets
    inputs.numOfTargets = result.targets.size.toDouble()
  }
}
