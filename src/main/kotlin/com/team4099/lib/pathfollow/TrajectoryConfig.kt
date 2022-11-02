package com.team4099.lib.pathfollow

import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint

data class TrajectoryConfig(
  val maxLinearVelocity: LinearVelocity,
  val maxLinearAcceleration: LinearAcceleration,
  val maxAngularVelocity: AngularVelocity,
  val maxAngularAcceleration: AngularAcceleration,
  val constraints: List<TrajectoryConstraint> = listOf()
)
