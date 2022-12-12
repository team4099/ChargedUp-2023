package com.team4099.lib.pathfollow

import com.team4099.lib.interpolate
import com.team4099.lib.units.base.Time

/**
 * A wrapper around the WPILib trajectory class that handles smooth heading changes for holonomic
 * drivetrains.
 */
class Trajectory(private val states: List<TrajectoryState>) {

  val startTime: Time
    get() = states[0].timestamp
  val endTime: Time
    get() = states[states.size - 1].timestamp

  val startingPose = states[0].pose
  val endingPose = states[states.size - 1].pose

  fun sample(time: Time): TrajectoryState {
    if (time <= startTime) {
      return states[0]
    }

    if (time >= endTime) {
      return states[states.size - 1]
    }

    var low = 1
    var high = states.size - 1

    // Binary search to find the closest timestamp
    while (low != high) {
      val mid = (low + high) / 2
      if (states[mid].timestamp < time) {
        low = mid + 1
      } else {
        high = mid
      }
    }

    val lowState = states[low - 1]
    val highState = states[low]
    val lerpScalar = (time - lowState.timestamp) / (highState.timestamp - lowState.timestamp)

    return TrajectoryState(
      time,
      interpolate(lowState.pose, highState.pose, lerpScalar),
      interpolate(lowState.curvature, highState.curvature, lerpScalar),
      interpolate(lowState.linearVelocity, highState.linearVelocity, lerpScalar),
      interpolate(lowState.linearAcceleration, highState.linearAcceleration, lerpScalar)
    )
    //        interpolate(lowState.angularVelocity, highState.angularVelocity, lerpScalar),
    //        interpolate(lowState.angularAcceleration, highState.angularAcceleration, lerpScalar)
  }

  //  private val path: Path,
  //  private val startVelocity: LinearVelocity,
  //  private val endVelocity: LinearVelocity,
  //  private val trajectoryConfig: TrajectoryConfig

  //  public fun convertToTrajectory(pathPlannerTrajectory: PathPlannerTrajectory):
  // edu.wpi.first.wpilibj.trajectory.Trajectory{
  //    var traj: Trajectory = edu.wpi.first.wpilibj.trajectory.Trajectory(
  //      e),
  //    pathPlannerTrajectory.
  //    )
  //  }
}
