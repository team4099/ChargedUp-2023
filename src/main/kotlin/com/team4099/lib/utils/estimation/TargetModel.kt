package com.team4099.lib.utils.estimation

import java.util.stream.Collectors
import com.team4099.lib.utils.util.CameraTargetRelation
import com.team4099.lib.utils.estimation.TargetModel
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import java.lang.IllegalArgumentException
import java.util.ArrayList

/**
 * Describes the shape of the target
 */
class TargetModel(cornerOffsets: List<Translation3d>?, widthMeters: Double, heightMeters: Double) {
  /**
   * Translations of this target's corners relative to its pose
   */
  @JvmField
  val cornerOffsets: List<Translation3d>
  var isPlanar = false
  val widthMeters: Double
  val heightMeters: Double
  val areaSqMeters: Double

  /**
   * This target's corners offset from its field pose.
   */
  fun getFieldCorners(targetPose: Pose3d): List<Translation3d> {
    return cornerOffsets.stream()
      .map { t: Translation3d? -> targetPose.plus(Transform3d(t, Rotation3d())).translation }
      .collect(Collectors.toList())
  }

  /**
   * This target's corners offset from its field pose, which is facing the camera.
   */
  fun getAgnosticFieldCorners(cameraPose: Pose3d?, targetPose: Pose3d): List<Translation3d> {
    val rel = CameraTargetRelation(cameraPose!!, targetPose)
    // this target's pose but facing the camera pose
    val facingPose = Pose3d(
      targetPose.translation,
      Rotation3d(0.0, rel.camToTargPitch.radians, rel.camToTargYaw.radians)
    )
    // find field corners based on this model's width/height if it was facing the camera
    return ofPlanarRect(widthMeters, heightMeters)
      .getFieldCorners(facingPose)
  }

  override fun equals(obj: Any?): Boolean {
    if (this === obj) return true
    if (obj is TargetModel) {
      val o = obj
      return cornerOffsets == o.cornerOffsets && widthMeters == o.widthMeters && heightMeters == o.heightMeters && areaSqMeters == o.areaSqMeters
    }
    return false
  }

  companion object {
    /**
     * Creates a rectangular, planar target model given the width and height.
     */
    @JvmStatic
    fun ofPlanarRect(widthMeters: Double, heightMeters: Double): TargetModel {
      // 4 corners of rect with its pose as origin
      return TargetModel(
        java.util.List.of( // this order is relevant for solvePNP
          Translation3d(0.0, -widthMeters / 2.0, heightMeters / 2.0),
          Translation3d(0.0, widthMeters / 2.0, heightMeters / 2.0),
          Translation3d(0.0, widthMeters / 2.0, -heightMeters / 2.0),
          Translation3d(0.0, -widthMeters / 2.0, -heightMeters / 2.0)
        ),
        widthMeters, heightMeters
      )
    }

    /**
     * Creates a spherical target which has similar dimensions when viewed from any angle.
     */
    fun ofSphere(diameterMeters: Double): TargetModel {
      // to get area = PI*r^2
      val assocSideLengths = Math.sqrt(Math.PI) * (diameterMeters / 2.0)
      return TargetModel(null, assocSideLengths, assocSideLengths)
    }
  }

  init {
    var cornerOffsets = cornerOffsets
    if (cornerOffsets == null || cornerOffsets.size <= 2) {
      cornerOffsets = ArrayList()
      isPlanar = false
    } else {
      var cornersPlanar = true
      for (corner in cornerOffsets) {
        if (corner.x != 0.0) cornersPlanar = false
      }
      require(!(cornerOffsets.size != 4 || !cornersPlanar)) {
        String.format(
          "Supplied target corners (%s) must total 4 and be planar (all X == 0).",
          cornerOffsets.size
        )
      }
      isPlanar = true
    }
    this.cornerOffsets = cornerOffsets
    this.widthMeters = widthMeters
    this.heightMeters = heightMeters
    areaSqMeters = widthMeters * heightMeters
  }
}
