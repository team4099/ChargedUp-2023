package com.team4099.lib.pathfollow

import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity

data class TrajectoryConfig(
  val maxLinearVelocity: LinearVelocity,
  val maxLinearAcceleration: LinearAcceleration,
  val maxAngularVelocity: AngularVelocity,
  val maxAngularAcceleration: AngularAcceleration,
  val constraints: List<TrajectoryConstraint> = listOf()
)
