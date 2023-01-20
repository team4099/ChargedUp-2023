package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.vision.TargetCorner
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import kotlin.math.absoluteValue

class Camera(val io: CameraIO) : SubsystemBase() {
  val inputs = CameraIO.CameraInputs()

  var detectedAprilTagIds = mutableListOf<Int>()

  var bestTransforms = mutableListOf<Transform3d>()
  var altTransforms = mutableListOf<Transform3d>()

  var timestamp: Time = 0.0.seconds

  var stdevs = mutableListOf<Triple<Double, Double, Double>>()

  override fun periodic() {
    io.updateInputs(inputs)

    val corners = mutableListOf<TargetCorner>()
    val knownTags = mutableListOf<Int>()
    val bestTransformResult = mutableListOf<Transform3d>()
    val altTransformResult = mutableListOf<Transform3d>()
    val resultTimeStamp: Time = inputs.visionResult.timestamp

    for (target in inputs.visionResult.targets) {
      stdevs.add(
        Triple(
          1 / (0.01 * target.area) + (target.yaw - 90.degrees).inDegrees.absoluteValue / 90,
          1 / (0.01 * target.area) + (target.yaw - 90.degrees).inDegrees.absoluteValue / 90,
          (target.yaw - 90.degrees).inDegrees.absoluteValue / 100
        )
      )
      knownTags.add(target.fiducialID)

      corners.addAll(target.targetCorners)

      // getting transforms
      val camToBest = target.bestTransform
      val camToAlt = target.altTransform

      bestTransformResult.add(camToBest)
      altTransformResult.add(camToAlt)
    }

    detectedAprilTagIds = knownTags
    bestTransforms = bestTransformResult
    altTransforms = altTransformResult
    timestamp = resultTimeStamp
  }
}
