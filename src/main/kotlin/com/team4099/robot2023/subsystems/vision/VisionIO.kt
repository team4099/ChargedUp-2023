package com.team4099.robot2023.subsystems.vision

import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.Camera
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import org.team4099.lib.geometry.Transform3d

interface VisionIO {
  val visionCameras: List<Camera>
    get() =
      listOf(
        Camera(
          object : CameraIO {
            override val cameraName = VisionConstants.FRONT_CAMERA_NAME
            override val transformToRobot: Transform3d = Transform3d()
          })
      )
}
