package com.team4099.robot2023.util

import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.RotationSequence
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.config.constants.FieldConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation2dWPILIB
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.sin

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in [FieldConstants] are stored with the origin at the rightmost point on the blue alliance
 * wall.
 */
object AllianceFlipUtil {
  /** Flips a translation to the correct side of the field based on the current alliance color. */
  fun apply(translation: Translation2d, force: Boolean = false): Translation2d {
    return if (shouldFlip() || force) {
      Translation2d(FieldConstants.fieldLength - translation.x, translation.y)
    } else {
      translation
    }
  }

  fun apply(translation: Translation2dWPILIB): Translation2dWPILIB {
    return if (shouldFlip()) {
      Translation2dWPILIB(FieldConstants.fieldLength.inMeters - translation.x, translation.y)
    } else {
      translation
    }
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  fun apply(xCoordinate: Length): Length {
    return if (shouldFlip()) {
      FieldConstants.fieldLength - xCoordinate
    } else {
      xCoordinate
    }
  }

  /** Flips a rotation based on the current alliance color. */
  fun apply(rotation: Rotation2d): Rotation2d {
    return if (shouldFlip()) {
      Rotation2d(-rotation.cos, rotation.sin)
    } else {
      rotation
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  fun apply(pose: Pose2d): Pose2d {
    return if (shouldFlip()) {
      Pose2d(
        FieldConstants.fieldLength - pose.x, pose.y, Angle(-pose.rotation.cos, pose.rotation.sin)
      )
    } else {
      pose
    }
  }

  fun apply(waypoint: Waypoint): Waypoint {
    return if (shouldFlip()) {
      Waypoint(apply(waypoint.translation), waypoint.driveRotation, waypoint.holonomicRotation)
    } else {
      waypoint
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  fun apply(state: Trajectory.State): Trajectory.State {
    return if (shouldFlip()) {
      Trajectory.State(
        state.timeSeconds,
        state.velocityMetersPerSecond,
        state.accelerationMetersPerSecondSq,
        Pose2d(
          FieldConstants.fieldLength - state.poseMeters.x.meters,
          state.poseMeters.y.meters,
          Angle(-state.poseMeters.rotation.cos, state.poseMeters.rotation.sin)
        )
          .pose2d,
        -state.curvatureRadPerMeter
      )
    } else {
      state
    }
  }

  fun apply(zone: Zone2d, force: Boolean = false): Zone2d {
    return if (shouldFlip() || force) {
      Zone2d(zone.vertices.map { AllianceFlipUtil.apply(it, force) })
    } else {
      zone
    }
  }

  /** Flips a rotation sequence state based on the current alliance color. */
  fun apply(state: RotationSequence.State): RotationSequence.State {
    return if (shouldFlip()) {
      RotationSequence.State(
        Rotation2d(-state.position.cos, state.position.sin), -state.velocityRadiansPerSec
      )
    } else {
      state
    }
  }

  private fun shouldFlip(): Boolean {
    return DriverStation.getAlliance().get() == Alliance.Red
  }
}
