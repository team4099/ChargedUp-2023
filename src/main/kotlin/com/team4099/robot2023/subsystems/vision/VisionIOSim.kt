package com.team4099.robot2023.subsystems.vision

import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.Camera
import com.team4099.robot2023.subsystems.vision.camera.CameraIOSim

object VisionIOSim : VisionIO {
  override val visionCameras: List<Camera>
    get() =
      listOf(
        Camera(
          CameraIOSim(
            VisionConstants.CAMERA_TRANSFORMS[0], VisionConstants.FRONT_CAMERA_NAME
          )
        )
      )
}
