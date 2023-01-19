package com.team4099.lib.sim.vision

import kotlin.jvm.JvmOverloads
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import java.util.stream.Collectors

/**
 * Represents a transformation that first rotates a pose around the origin,
 * and then translates it.
 */
class RotTrlTransform3d
/**
 * A rotation-translation transformation.
 *
 *
 * Applying this transformation to poses will preserve their current origin-to-pose
 * transform as if the origin was transformed by these components.
 *
 * @param rot The rotation component
 * @param trl The translation component
 */ @JvmOverloads constructor(
  /** The rotation component of this transformation  */
  val rotation: Rotation3d = Rotation3d(),
  /** The translation component of this transformation  */
  val translation: Translation3d = Translation3d()
) {

  /**
   * Creates a rotation-translation transformation from a Transform3d.
   *
   *
   * Applying this transformation to poses will preserve their current origin-to-pose
   * transform as if the origin was transformed by these components.
   *
   * @param trf The origin transformation
   */
  constructor(trf: Transform3d) : this(trf.rotation, trf.translation) {}

  /**
   * The inverse of this transformation. Applying the inverse will "undo" this transformation.
   */
  fun inverse(): RotTrlTransform3d {
    val inverseRot = rotation.unaryMinus()
    val inverseTrl = translation.rotateBy(inverseRot).unaryMinus()
    return RotTrlTransform3d(inverseRot, inverseTrl)
  }

  /** This transformation as a Transform3d (as if of the origin)  */
  val transform: Transform3d
    get() = Transform3d(translation, rotation)

  fun apply(trl: Translation3d?): Translation3d {
    return apply(Pose3d(trl, Rotation3d())).translation
  }

  fun applyTrls(trls: List<Translation3d?>): List<Translation3d> {
    return trls.stream().map { t: Translation3d? -> apply(t) }
      .collect(Collectors.toList())
  }

  fun apply(pose: Pose3d): Pose3d {
    return Pose3d(
      pose.translation.rotateBy(rotation).plus(translation),
      pose.rotation.plus(rotation)
    )
  }

  fun applyPoses(poses: List<Pose3d>): List<Pose3d> {
    return poses.stream().map { p: Pose3d -> apply(p) }
      .collect(Collectors.toList())
  }

  companion object {
    /**
     * The rotation-translation transformation that makes poses in the world
     * consider this pose as the new origin, or change the basis to this pose.
     *
     * @param pose The new origin
     */
    @JvmStatic
    fun makeRelativeTo(pose: Pose3d): RotTrlTransform3d {
      return RotTrlTransform3d(pose.rotation, pose.translation).inverse()
    }
  }
}
