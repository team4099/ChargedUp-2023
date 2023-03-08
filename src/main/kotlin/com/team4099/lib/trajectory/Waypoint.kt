package com.team4099.lib.trajectory

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.ErrorMessages
import java.util.Optional

/** A trajectory waypoint, including a translation and optional drive/holonomic rotations. */
class Waypoint {
  /** Returns the translation component of the waypoint. */
  val translation: Translation2d
  private val driveRotation: Rotation2d?
  private val holonomicRotation: Rotation2d?

  /**
   * Constructs a Waypoint with a translation, drive rotation, and holonomic rotation.
   *
   * @param translation Waypoint position (required)
   * @param heading Drive velocity rotation (optional, can be null)
   * @param holonomicRotation Holonomic rotation (optional, can be null)
   */
  constructor(
    translation: Translation2d,
    heading: Rotation2d? = null,
    holonomicRotation: Rotation2d? = null,
  ) {
    this.translation = ErrorMessages.requireNonNullParam(translation, "translation", "Waypoint")
    this.driveRotation = heading
    this.holonomicRotation = holonomicRotation
  }
  /**
   * Constructs a Waypoint with a translation (but no drive or holonomic rotation).
   *
   * @param translation Waypoint position (required)
   */
  /** Constructs a Waypoint at the origin and without a drive or holonomic rotation. */
  @JvmOverloads
  constructor(translation: Translation2d = Translation2d()) {
    this.translation = ErrorMessages.requireNonNullParam(translation, "translation", "Waypoint")
    driveRotation = null
    holonomicRotation = null
  }

  /**
   * Returns the drive rotation component of the waypoint (or an empty optional if not specified).
   */
  fun getDriveRotation(): Optional<Rotation2d> {
    return Optional.ofNullable(driveRotation)
  }

  /**
   * Returns the holonomic rotation component of the waypoint (or an empty optional if not
   * specified).
   */
  fun getHolonomicRotation(): Optional<Rotation2d> {
    return Optional.ofNullable(holonomicRotation)
  }

  companion object {
    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the drive rotation)
     */
    fun fromDifferentialPose(pose: Pose2d): Waypoint {
      ErrorMessages.requireNonNullParam(pose, "pose", "Waypoint")
      return Waypoint(pose.translation, pose.rotation, null)
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the drive rotation)
     * @param holonomicRotation Holonomic rotation
     */
    fun fromDifferentialPose(pose: Pose2d, holonomicRotation: Rotation2d?): Waypoint {
      ErrorMessages.requireNonNullParam(pose, "pose", "Waypoint")
      return Waypoint(pose.translation, pose.rotation, holonomicRotation)
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the holonomic rotation)
     */
    fun fromHolonomicPose(pose: Pose2d): Waypoint {
      ErrorMessages.requireNonNullParam(pose, "pose", "Waypoint")
      return Waypoint(pose.translation, null, pose.rotation)
    }

    /**
     * Constucts a Waypoint based on a pose.
     *
     * @param pose Source pose (where the rotation describes the holonomic rotation)
     * @param driveRotation Drive rotation
     */
    fun fromHolonomicPose(pose: Pose2d, driveRotation: Rotation2d?): Waypoint {
      ErrorMessages.requireNonNullParam(pose, "pose", "Waypoint")
      return Waypoint(pose.translation, driveRotation, pose.rotation)
    }
  }
}
