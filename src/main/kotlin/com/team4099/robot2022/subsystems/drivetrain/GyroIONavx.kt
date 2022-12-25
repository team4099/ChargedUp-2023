package com.team4099.robot2022.subsystems.drivetrain

import com.kauailabs.navx.frc.AHRS
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.SerialPort
import kotlin.math.IEEErem

object GyroIONavx : GyroIO {
  private val gyro = AHRS(SerialPort.Port.kMXP)

  init {
    gyro.calibrate()
  }

  var gyroYawOffset: Angle = 0.0.degrees
  var gyroPitchOffset: Angle = 0.0.degrees
  var gyroRollOffset: Angle = 0.0.degrees

  /** The current angle of the drivetrain. */
  val gyroYaw: Angle
    get() {
      if (gyro.isConnected) {
        var rawYaw = gyro.angle + gyroYawOffset.inDegrees
        rawYaw += DrivetrainConstants.GYRO_RATE_COEFFICIENT * gyro.rate
        return rawYaw.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroPitch: Angle
    get() {
      if (gyro.isConnected) {
        val rawPitch = gyro.pitch + gyroPitchOffset.inDegrees
        return rawPitch.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
      }
    }

  val gyroRoll: Angle
    get() {
      if (gyro.isConnected) {
        val rawRoll = gyro.roll + gyroRollOffset.inDegrees
        return rawRoll.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
      }
    }

  val gyroYawRate: AngularVelocity
    get() {
      if (gyro.isConnected) {
        return gyro.rate.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.gyroYaw = gyroYaw
    inputs.gyroVelocity = gyroYawRate
    inputs.gyroPitch = gyroPitch
    inputs.gyroRoll = gyroRollOffset

    inputs.gyroConnected = gyro.isConnected
  }

  override fun zeroGyroYaw(toAngle: Angle) {
    gyroYawOffset = (toAngle.inDegrees - gyro.angle).degrees
  }

  override fun zeroGyroPitch(toAngle: Angle) {
    gyroPitchOffset = (toAngle.inDegrees - gyro.pitch).degrees
  }

  override fun zeroGyroRoll(toAngle: Angle) {
    gyroRollOffset = (toAngle.inDegrees - gyro.roll).degrees
  }
}
