package com.team4099.lib.kinematics

import com.team4099.lib.geometry.Rotation2d
import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.inRadiansPerSecondPerSecond
import com.team4099.lib.units.perSecond
import edu.wpi.first.math.kinematics.ChassisSpeeds as ChassisSpeedsWPILIB

class ChassisAccels(
  val ax: LinearAcceleration,
  val ay: LinearAcceleration,
  val alpha: AngularAcceleration
) {
  constructor() :
    this(
      0.0.meters.perSecond.perSecond,
      0.0.meters.perSecond.perSecond,
      0.0.radians.perSecond.perSecond
    ) {}

  val chassisAccelsWPILIB =
    ChassisSpeedsWPILIB(
      ax.inMetersPerSecondPerSecond,
      ay.inMetersPerSecondPerSecond,
      alpha.inRadiansPerSecondPerSecond
    )

  companion object {
    fun fromFieldRelativeAccels(
      ax: LinearAcceleration,
      ay: LinearAcceleration,
      alpha: AngularAcceleration,
      robotAngle: Rotation2d
    ): ChassisAccels {
      return ChassisAccels(
        ax * robotAngle.m_cos + ay * robotAngle.m_sin,
        -ax * robotAngle.m_sin + ay * robotAngle.m_cos,
        alpha
      )
    }

    fun fromFieldRelativeAccels(
      fieldRelativeAccels: ChassisAccels,
      robotAngle: Rotation2d
    ): ChassisAccels {
      return fromFieldRelativeAccels(
        fieldRelativeAccels.ax, fieldRelativeAccels.ay, fieldRelativeAccels.alpha, robotAngle
      )
    }
  }
}
