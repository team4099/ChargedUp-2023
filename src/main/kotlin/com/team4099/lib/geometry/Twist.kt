package com.team4099.lib.geometry

import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.Time
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.geometry.Twist2d

data class Twist(val dx: LinearVelocity, val dy: LinearVelocity, val dtheta: AngularVelocity) {
  constructor(
    translationSpeeds: Pair<LinearVelocity, LinearVelocity>,
    theta: AngularVelocity
  ) : this(translationSpeeds.first, translationSpeeds.second, theta)

  constructor(
    twist: Twist2d,
    timestep: Time
  ) : this(
    (twist.dx / timestep.inSeconds).meters.perSecond,
    (twist.dy / timestep.inSeconds).meters.perSecond,
    (twist.dtheta / timestep.inSeconds).radians.perSecond
  )

  fun toTwist2d(timestep: Time): Twist2d {
    return Twist2d(
      dx.inMetersPerSecond * timestep.inSeconds,
      dy.inMetersPerSecond * timestep.inSeconds,
      dtheta.inRadiansPerSecond * timestep.inSeconds
    )
  }
}
