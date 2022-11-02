package com.team4099.lib.units.derived

import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import com.team4099.lib.units.base.Length
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI

object Radian : UnitKey

typealias Angle = Value<Radian>

val Double.radians: Angle
  get() = Angle(this)

val Double.degrees: Angle
  get() = Angle(Math.toRadians(this))

val Double.rotations: Angle
  get() = Angle(this * 2 * Math.PI)

val Rotation2d.angle: Angle
  get() = Angle(this.radians)

val Number.radians: Angle
  get() = toDouble().radians

val Number.degrees: Angle
  get() = toDouble().degrees

val Number.rotations: Angle
  get() = toDouble().rotations

val Angle.inDegrees: Double
  get() = Math.toDegrees(value)

val Angle.inRadians: Double
  get() = value

val Angle.inRotations: Double
  get() = value / (2 * PI)

val Angle.inRotation2ds: Rotation2d
  get() = Rotation2d(value)

val Angle.sin: Double
  get() = kotlin.math.sin(value)

val Angle.cos: Double
  get() = kotlin.math.cos(value)

val Angle.tan: Double
  get() = kotlin.math.tan(value)

operator fun AngularVelocity.times(o: Length): LinearVelocity = (o * inRadiansPerSecond).perSecond
