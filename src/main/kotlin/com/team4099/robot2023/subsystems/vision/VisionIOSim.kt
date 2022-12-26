package com.team4099.robot2023.subsystems.vision

import org.photonvision.PhotonCamera
import org.photonvision.SimPhotonCamera

object VisionIOSim : VisionIO {

  val camera1: PhotonCamera = SimPhotonCamera("")

  override fun updateInputs(inputs: VisionIO.VisionInputs) {
    super.updateInputs(inputs)
  }
}
