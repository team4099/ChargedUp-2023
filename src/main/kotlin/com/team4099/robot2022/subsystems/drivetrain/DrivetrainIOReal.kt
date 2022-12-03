package com.team4099.robot2022.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.kauailabs.navx.frc.AHRS
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.config.constants.Constants.Universal.CANIVORE_NAME
import com.team4099.robot2022.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.SerialPort
import java.lang.Math.PI
import kotlin.math.IEEErem

object DrivetrainIOReal : DrivetrainIO {
  override fun getSwerveModules(): List<SwerveModule> {
    return listOf(
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.FRONT_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.FRONT_LEFT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.FRONT_LEFT_MODULE_ZERO,
          "Front Left Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.FRONT_RIGHT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.FRONT_RIGHT_MODULE_ZERO,
          "Front Right Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.BACK_LEFT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.BACK_LEFT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.BACK_LEFT_MODULE_ZERO,
          "Back Left Wheel"
        )
      ),
      SwerveModule(
        SwerveModuleIOReal(
          TalonFX(Constants.Drivetrain.BACK_RIGHT_STEERING_ID, CANIVORE_NAME),
          TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID, CANIVORE_NAME),
          AnalogPotentiometer(
            Constants.Drivetrain.BACK_RIGHT_ANALOG_POTENTIOMETER, 2 * PI, 0.0
          ),
          DrivetrainConstants.BACK_RIGHT_MODULE_ZERO,
          "Back Right Wheel"
        )
      )
    )
  }

  private val gyro = AHRS(SerialPort.Port.kMXP)

  init {
    gyro.calibrate()
  }

  var gyroYawOffset: Angle = 0.0.degrees
  var gyroPitchOffset: Angle = 0.0.degrees

  /** The current angle of the drivetrain. */
  val gyroYaw: Angle
    get() {
      if (gyro.isConnected) {
        var rawYaw = gyro.angle + gyroYawOffset.inDegrees
        rawYaw += DrivetrainConstants.GYRO_RATE_COEFFICIENT * gyro.rate
        return rawYaw.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
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

  val gyroYawRate: AngularVelocity
    get() {
      if (gyro.isConnected) {
        return gyro.rate.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  override fun updateInputs(inputs: DrivetrainIO.DrivetrainIOInputs) {
    inputs.gyroYaw = gyroYaw
    inputs.gyroVelocity = gyroYawRate
    inputs.gyroPitch = gyroPitch

    inputs.gyroConnected = gyro.isConnected
  }

  override fun zeroGyroYaw(toAngle: Angle) {
    gyroYawOffset = (toAngle.inDegrees - gyro.angle).degrees
  }

  override fun zeroGyroPitch(toAngle: Angle) {
    gyroPitchOffset = (toAngle.inDegrees - gyro.pitch).degrees
  }
}
