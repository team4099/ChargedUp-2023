package com.team4099.lib.pathfollow

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.trajectory.TrajectoryParameterizer
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.unitless
import java.util.TreeMap
import kotlin.math.absoluteValue

fun trajectoryFromPath(path: Path, trajectoryConfig: TrajectoryConfig): Trajectory {
  if (!path.built) path.build()

  val waypoints = path.holonomicWaypoints

  val wpilibStates =
    TrajectoryParameterizer.timeParameterizeTrajectory(
      path.splinePoints,
      trajectoryConfig.constraints,
      path.startingVelocity.magnitude.inMetersPerSecond,
      path.endingVelocity.magnitude.inMetersPerSecond,
      trajectoryConfig.maxLinearVelocity.inMetersPerSecond,
      trajectoryConfig.maxLinearAcceleration.inMetersPerSecondPerSecond,
      false
    )
      .states

  val holonomicWaypoints: TreeMap<Time, Angle> = TreeMap<Time, Angle>()
  var stateIndex = 0
  for (waypointIndex in waypoints.indices){
    val timestamp: Time = if (waypointIndex == 0){
      0.0.seconds
    } else if (waypointIndex == waypoints.size-1){
      wpilibStates.last().timeSeconds.seconds
    } else {
      while (wpilibStates[stateIndex].poseMeters.translation == waypoints[waypointIndex].translation){
        stateIndex++
      }
      wpilibStates[stateIndex].timeSeconds.seconds
    }

    if (waypoints[waypointIndex].rotation != null){
      holonomicWaypoints[timestamp] = waypoints[waypointIndex].rotation.angle
    }
  }

  val rotationSequence = RotationSequence(holonomicWaypoints)

  val states =
    wpilibStates.map{ state ->
      TrajectoryState(
        state.timeSeconds.seconds,
        Pose2d(Translation2d(state.poseMeters.translation), rotationSequence.sample(state.timeSeconds.seconds).angle),
        state.poseMeters.rotation.angle,
        state.velocityMetersPerSecond.meters.perSecond,
        state.accelerationMetersPerSecondSq.meters.perSecond.perSecond
      )
    }

  return Trajectory(states)
}

fun trajectoryFromPathPlanner(pathPlannerTrajectory: PathPlannerTrajectory): Trajectory {
  return Trajectory(
    pathPlannerTrajectory.states.map { state ->
      state as PathPlannerTrajectory.PathPlannerState
      TrajectoryState(
        state.timeSeconds.seconds,
        Pose2d(Translation2d(state.poseMeters.translation), state.holonomicRotation.angle),
        state.poseMeters.rotation.angle,
        state.velocityMetersPerSecond.meters.perSecond,
        state.accelerationMetersPerSecondSq.meters.perSecond.perSecond,
        angularVelocity = state.angularVelocityRadPerSec.radians.perSecond
      )
    }
  )
}
