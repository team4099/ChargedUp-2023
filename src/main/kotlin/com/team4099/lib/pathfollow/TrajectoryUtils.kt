package com.team4099.lib.pathfollow

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.trajectory.TrajectoryParameterizer
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond

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
          path.startingPose.rotation
        } else if (index == wpilibStates.size - 1) {
          path.endingPose.rotation
        } else {
          val tailMap = path.headingPointMap.tailMap(index)
          if (tailMap.size == 0) {
            path.endingPose.rotation
          } else {
            path.headingPointMap[tailMap.firstKey()]
          }
        }

      if (headingTarget == null) {
        headingTarget = path.endingPose.rotation
      }
        /*
         #TO DO

         Look at wpilib to see if we can generate a trapezoidal profile
          using max angular vel and acceleration

          Look back to this if rotation is jittery
        */
      TrajectoryState(
        state.timeSeconds.seconds,
        Pose2d(Translation2d(state.poseMeters.translation), headingTarget),
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
