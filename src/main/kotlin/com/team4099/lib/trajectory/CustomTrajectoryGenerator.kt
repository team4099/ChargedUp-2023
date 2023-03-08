package com.team4099.lib.trajectory

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import java.security.InvalidParameterException
import java.util.Optional
import java.util.TreeMap
import kotlin.collections.ArrayList

/** Generator for creating a drive trajectory and rotation sequence from a series of waypoints. */
class CustomTrajectoryGenerator {
  /** Returns the generated drive trajectory. */
  var driveTrajectory = Trajectory(listOf(Trajectory.State()))
    private set

  /** Returns the generated holonomic rotation sequence. */
  var holonomicRotationSequence = RotationSequence(TreeMap())
    private set

  /**
   * Generates a drive trajectory and holonomic rotation sequence from a series of waypoints,
   * combining quintic and cubic splines when necessary. Please note the following limitations:
   *
   * 1) The drive rotations for the start and end waypoints are required for trajectory generation.
   * If not specified by the user, these rotations will be created automatically based on the
   * position of the nearest waypoint.
   *
   * 2) Transitions between quintic and cubic splines may produce high accelerations due to
   * unrealistic changes in curvature. Please account for this effect when tuning feedback
   * controllers.
   *
   * 3) The holonomic rotation sequence attempts to minimize the angular acceleration between
   * points, but does not accept velocity or acceleration constraints as inputs. Ensure that the
   * change in rotation between adjacent waypoints is reasonable.
   *
   * 4) The robot's holonomic rotation always assumed to match its drive rotation when applying
   * kinematics constraints. Please use sufficient margins to allow for changes in angular position
   * and velocity.
   *
   * @param config Trajectory configuration
   * @param waypoints A series of waypoints
   */
  fun generate(config: TrajectoryConfig, waypoints: List<Waypoint>) {
    if (waypoints.size < 2) {
      throw InvalidParameterException(
        "Please include at least 2 waypoints to generate a trajectory."
      )
    }

    // Generate drive waypoints
    val driveTranslations = waypoints.map { it.translation }.toMutableList()
    val driveRotations = waypoints.map { it.getDriveRotation() }.toMutableList()
    driveRotations.removeAt(0)
    driveRotations.removeAt(driveRotations.size - 1)

    // Add first drive waypoint
    if (waypoints[0].getDriveRotation().isPresent) {
      driveRotations.add(0, waypoints[0].getDriveRotation())
    } else {
      driveRotations.add(
        0, Optional.of(waypoints[1].translation.minus(waypoints[0].translation).angle)
      )
    }

    // Add last drive waypoint
    if (waypoints[waypoints.size - 1].getDriveRotation().isPresent()) {
      driveRotations.add(waypoints[waypoints.size - 1].getDriveRotation())
    } else {
      driveRotations.add(
        Optional.of(
          waypoints[waypoints.size - 1].translation.minus(
            waypoints[waypoints.size - 2].translation
          )
            .angle
        )
      )
    }

    // Generate drive trajectory
    driveTrajectory = Trajectory()
    var firstSubTrajectory = true
    var nextQuintic = driveRotations[1].isPresent
    var index = 1
    var subTrajectoryStart = 0
    var subTrajectoryEnd = 0
    while (true) {
      var generateSubTrajectory = false
      val lastWaypoint = index == waypoints.size - 1
      if (nextQuintic) {
        if (lastWaypoint || driveRotations[index].isEmpty) { // Found a translation or end of
          // waypoints
          generateSubTrajectory = true
          subTrajectoryEnd = if (lastWaypoint) index else index - 1
        }
      } else {
        if (driveRotations[index].isPresent) { // Found a pose
          generateSubTrajectory = true
          subTrajectoryEnd = index
        }
      }
      if (generateSubTrajectory) {
        // Prepare sub-trajectory config
        val subConfig = copyConfig(config)
        if (!firstSubTrajectory) {
          subConfig.startVelocity =
            driveTrajectory.states[driveTrajectory.states.size - 1].velocityMetersPerSecond
        }
        if (!lastWaypoint) {
          subConfig.endVelocity = subConfig.maxVelocity
        }
        firstSubTrajectory = false

        // Generate sub-trajectory
        driveTrajectory =
          if (nextQuintic) {
            val quinticWaypoints: MutableList<Pose2d> = ArrayList()
            for (i in subTrajectoryStart until subTrajectoryEnd + 1) {
              quinticWaypoints.add(Pose2d(driveTranslations[i], driveRotations[i].get()))
            }
            driveTrajectory.concatenate(
              TrajectoryGenerator.generateTrajectory(quinticWaypoints, subConfig)
            )
          } else {
            val cubicInteriorWaypoints: MutableList<Translation2d> = ArrayList()
            for (i in subTrajectoryStart + 1 until subTrajectoryEnd) {
              cubicInteriorWaypoints.add(driveTranslations[i])
            }
            driveTrajectory.concatenate(
              TrajectoryGenerator.generateTrajectory(
                Pose2d(
                  driveTranslations[subTrajectoryStart],
                  driveRotations[subTrajectoryStart].get()
                ),
                cubicInteriorWaypoints,
                Pose2d(
                  driveTranslations[subTrajectoryEnd],
                  driveRotations[subTrajectoryEnd].get()
                ),
                subConfig
              )
            )
          }

        // Break if complete
        if (lastWaypoint) {
          break
        }

        // Prepare for next trajectory
        nextQuintic = !nextQuintic
        subTrajectoryStart = subTrajectoryEnd
      }
      index++
    }

    // Find holonmic waypoints
    val holonomicWaypoints = TreeMap<Double, Rotation2d>()
    var stateIndex = 0
    for (waypointIndex in driveTranslations.indices) {
      var timestamp: Double
      timestamp =
        if (waypointIndex == 0) {
          0.0
        } else if (waypointIndex == driveTranslations.size - 1) {
          driveTrajectory.totalTimeSeconds
        } else {
          while (driveTrajectory.states[stateIndex].poseMeters.translation !=
            driveTranslations[waypointIndex]
          ) {
            stateIndex++
          }
          driveTrajectory.states[stateIndex].timeSeconds
        }
      if (waypoints[waypointIndex].getHolonomicRotation().isPresent()) {
        holonomicWaypoints[timestamp] = waypoints[waypointIndex].getHolonomicRotation().get()
      }
    }
    holonomicRotationSequence = RotationSequence(holonomicWaypoints)
  }

  private fun copyConfig(config: TrajectoryConfig): TrajectoryConfig {
    val newConfig = TrajectoryConfig(config.maxVelocity, config.maxAcceleration)
    newConfig.addConstraints(config.constraints)
    newConfig.startVelocity = config.startVelocity
    newConfig.endVelocity = config.endVelocity
    newConfig.isReversed = config.isReversed
    return newConfig
  }
}
