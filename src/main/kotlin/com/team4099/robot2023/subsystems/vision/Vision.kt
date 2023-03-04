package com.team4099.robot2023.subsystems.vision

import com.team4099.robot2023.config.constants.VisionConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.seconds

class Vision(cameras: VisionIO) : SubsystemBase() {
  val ambiguityThreshold = 0.15
  val targetLogTime = 0.05.seconds
  val cameraPoses = VisionConstants.CAMERA_TRANSFORMS
}
