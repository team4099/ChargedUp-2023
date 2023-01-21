package com.team4099.lib.vision

import edu.wpi.first.math.util.Units
import org.photonvision.targeting.PhotonTrackedTarget
import org.team4099.lib.geometry.Transform3d

data class VisionTarget(
  val yaw: Double,
  val pitch: Double,
  val area: Double,
  val skew: Double,
  val fiducialID: Int,
  val bestTransform: edu.wpi.first.math.geometry.Transform3d,
  val altTransform: edu.wpi.first.math.geometry.Transform3d,
  val poseAmbiguity: Double,
  val targetCorners: List<TargetCorner>
) {

  constructor(
    photonTrackedTarget: PhotonTrackedTarget
  ) : this(
    photonTrackedTarget.yaw,
    photonTrackedTarget.pitch,
    photonTrackedTarget.area,
    photonTrackedTarget.skew,
    photonTrackedTarget.fiducialId,
    photonTrackedTarget.bestCameraToTarget,
    photonTrackedTarget.alternateCameraToTarget,
    photonTrackedTarget.poseAmbiguity,
    photonTrackedTarget.corners.map { TargetCorner(it) }
  )

  // limelight constructor
  constructor(
    yaw: Double,
    pitch: Double,
    area: Double,
    fiducialID: Int,
    transform3d: edu.wpi.first.math.geometry.Transform3d,
    targetCorners: List<TargetCorner>
  ) : this(
    Units.degreesToRadians(yaw),
    Units.degreesToRadians(pitch),
    area,
    0.0,
    fiducialID,
    transform3d,
    transform3d,
    0.0,
    targetCorners
  ) {}
}
