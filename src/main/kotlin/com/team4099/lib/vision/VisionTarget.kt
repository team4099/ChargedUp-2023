package com.team4099.lib.vision

import org.photonvision.targeting.PhotonTrackedTarget
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians

data class VisionTarget(val yaw: Angle, val pitch: Angle, val area: Double, val skew: Angle, val fiducialID: Int, val bestTransform: Transform3d, val altTransform: Transform3d, val poseAmbiguity: Double, val targetCorners: List<TargetCorner>){

  constructor(
    photonTrackedTarget: PhotonTrackedTarget
  ): this(
    photonTrackedTarget.yaw.radians, photonTrackedTarget.pitch.radians, photonTrackedTarget.area, photonTrackedTarget.skew.radians, photonTrackedTarget.fiducialId, Transform3d(photonTrackedTarget.bestCameraToTarget), Transform3d(photonTrackedTarget.alternateCameraToTarget), photonTrackedTarget.poseAmbiguity, photonTrackedTarget.corners.map { TargetCorner(it) }
  )

  // limelight constructor
  constructor(
     yaw: Angle,  pitch: Angle,  area: Double,  fiducialID: Int,  transform3d: Transform3d, targetCorners: List<TargetCorner>
  ): this(yaw, pitch, area, 0.0.degrees, fiducialID, transform3d, transform3d, 0.0, targetCorners) {}

}
