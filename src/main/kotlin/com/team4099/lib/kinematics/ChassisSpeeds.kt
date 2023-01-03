package com.team4099.lib.kinematics

import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.Value
import com.team4099.lib.units.Velocity
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Radian
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPIChassisSpeeds

class ChassisSpeeds(val vx: LinearVelocity, val vy: LinearVelocity, val omega: AngularVelocity) {
  constructor(
    chassisSpeeds: WPIChassisSpeeds
  ) : this(
    chassisSpeeds.vxMetersPerSecond.meters.perSecond,
    chassisSpeeds.vyMetersPerSecond.meters.perSecond,
    chassisSpeeds.omegaRadiansPerSecond.radians.perSecond
  ) {}

  constructor() : this(0.0.meters.perSecond, 0.0.meters.perSecond, 0.0.radians.perSecond) {}

  val chassisSpeedsWPILIB =
    WPIChassisSpeeds(vx.inMetersPerSecond, vy.inMetersPerSecond, omega.inRadiansPerSecond)

  companion object {
    fun fromFieldRelativeSpeeds(
      vx: Value<Velocity<Meter>>,
      vy: Value<Velocity<Meter>>,
      omega: Value<Velocity<Radian>>,
      robotAngle: Rotation2d
    ): ChassisSpeeds {
      return ChassisSpeeds(
        vx * robotAngle.m_cos + vy * robotAngle.m_sin,
        -vx * robotAngle.m_sin + vy * robotAngle.m_cos,
        omega
      )
    }

    fun fromFieldRelativeSpeeds(
      fieldRelativeSpeeds: ChassisSpeeds,
      robotAngle: Rotation2d
    ): ChassisSpeeds {
      return fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vx, fieldRelativeSpeeds.vy, fieldRelativeSpeeds.omega, robotAngle
      )
    }
  }
}
