package com.team4099.lib.pathfollow

import com.team4099.lib.geometry.Pose2d
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.perSecond

data class TrajectoryState(
  val timestamp: Time,
  val pose: Pose2d,
  val curvature: Angle,
  val linearVelocity: LinearVelocity,
  val linearAcceleration: LinearAcceleration,
  val angularVelocity: AngularVelocity = 0.radians.perSecond
//  val angularAcceleration: AngularAcceleration
)
