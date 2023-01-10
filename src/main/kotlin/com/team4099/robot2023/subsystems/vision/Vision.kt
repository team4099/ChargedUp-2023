package com.team4099.robot2023.subsystems.vision

import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Vision(val io: VisionIO) : SubsystemBase() {
  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )
}
