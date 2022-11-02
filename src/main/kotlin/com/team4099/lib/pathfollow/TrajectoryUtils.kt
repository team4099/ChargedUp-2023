package com.team4099.lib.pathfollow

import com.pathplanner.lib.PathPlannerTrajectory
import com.team4099.lib.geometry.Pose
import com.team4099.lib.geometry.Translation
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.angle
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.trajectory.TrajectoryParameterizer

fun trajectoryFromPath(
  startVelocity: LinearVelocity,
  path: Path,
  endVelocity: LinearVelocity,
  trajectoryConfig: TrajectoryConfig
): Trajectory {
  if (!path.built) path.build()

  val wpilibStates =
    TrajectoryParameterizer.timeParameterizeTrajectory(
      path.splinePoints,
      trajectoryConfig.constraints,
      startVelocity.inMetersPerSecond,
      endVelocity.inMetersPerSecond,
      trajectoryConfig.maxLinearVelocity.inMetersPerSecond,
      trajectoryConfig.maxLinearAcceleration.inMetersPerSecondPerSecond,
      false
    )
      .states

  val states =
    wpilibStates.mapIndexed { index, state ->
      var headingTarget =
        if (index == 0) {
          path.startingPose.theta
        } else if (index == wpilibStates.size - 1) {
          path.endingPose.theta
        } else {
          val tailMap = path.headingPointMap.tailMap(index)
          if (tailMap.size == 0) {
            path.endingPose.theta
          } else {
            path.headingPointMap[tailMap.firstKey()]
          }
        }

      if (headingTarget == null) {
        headingTarget = path.endingPose.theta
      }
        /*
         #TO DO

         Look at wpilib to see if we can generate a trapezoidal profile
          using max angular vel and acceleration

          Look back to this if rotation is jittery
        */
      TrajectoryState(
        state.timeSeconds.seconds,
        Pose(Translation(state.poseMeters.translation), headingTarget),
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
        Pose(Translation(state.poseMeters.translation), state.holonomicRotation.angle),
        state.poseMeters.rotation.angle,
        state.velocityMetersPerSecond.meters.perSecond,
        state.accelerationMetersPerSecondSq.meters.perSecond.perSecond,
        angularVelocity = state.angularVelocity.angle.perSecond
      )
    }
  )
}
