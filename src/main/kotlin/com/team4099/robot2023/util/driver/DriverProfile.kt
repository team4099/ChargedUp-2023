package com.team4099.robot2023.util.driver

import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sign

abstract class DriverProfile(
  private val invertDrive: Boolean,
  private val invertRotation: Boolean,
  private val sensitivityDrivePowerConstant: Int,
  private val sensitivityRotationPowerConstant: Int,
  private val slowModeClamp: Double = 0.25
) {
  private val invertDriveMultiplier = if (invertDrive) -1 else 1
  private val invertRotationMultiplier = if (invertRotation) -1 else 1

  fun driveSpeedClampedSupplier(
    driveX: DoubleSupplier,
    driveY: DoubleSupplier,
    slowMode: BooleanSupplier
  ): Pair<LinearVelocity, LinearVelocity> {
    var xSpeedCurve = driveX.asDouble.pow(sensitivityDrivePowerConstant) * invertDriveMultiplier
    var ySpeedCurve = driveY.asDouble.pow(sensitivityDrivePowerConstant) * invertDriveMultiplier

    if (sensitivityDrivePowerConstant % 2 == 0) {
      xSpeedCurve *= sign(driveX.asDouble)
      ySpeedCurve *= sign(driveY.asDouble)
    }

    if (slowMode.asBoolean) {
      return Pair(
        DrivetrainConstants.DRIVE_SETPOINT_MAX * xSpeedCurve * slowModeClamp,
        DrivetrainConstants.DRIVE_SETPOINT_MAX * ySpeedCurve * slowModeClamp
      )
    } else {
      return Pair(
        DrivetrainConstants.DRIVE_SETPOINT_MAX * xSpeedCurve,
        DrivetrainConstants.DRIVE_SETPOINT_MAX * ySpeedCurve
      )
    }
  }

  fun rotationSpeedClampedSupplier(
    turn: DoubleSupplier,
    slowMode: BooleanSupplier
  ): AngularVelocity {
    var rotationSpeedCurve =
      turn.asDouble.pow(sensitivityDrivePowerConstant) * invertRotationMultiplier

    if (sensitivityRotationPowerConstant % 2 == 0) {
      rotationSpeedCurve *= sign(turn.asDouble)
    }

    if (slowMode.asBoolean) {
      return DrivetrainConstants.TURN_SETPOINT_MAX * rotationSpeedCurve * slowModeClamp
    } else {
      return DrivetrainConstants.TURN_SETPOINT_MAX * rotationSpeedCurve
    }
  }
}
