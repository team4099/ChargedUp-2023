package com.team4099.lib.pathfollow

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

data class TrajectoryState(
  val timestamp: Time,
  val pose: Pose2d,
  val curvature: Angle,
  val linearVelocity: LinearVelocity,
  val linearAcceleration: LinearAcceleration,
  val angularVelocity: AngularVelocity = 0.radians.perSecond
//  val angularAcceleration: AngularAcceleration
)
