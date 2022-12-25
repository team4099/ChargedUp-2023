package com.team4099.robot2022.subsystems.drivetrain

import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.celsius
import com.team4099.lib.units.base.inAmperes
import com.team4099.lib.units.base.inCelsius
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface SwerveModuleIO {
  class SwerveModuleIOInputs : LoggableInputs {
    var driveAppliedVoltage = 0.0.volts
    var steeringAppliedVoltage = 0.0.volts

    var driveStatorCurrent = 0.0.amps
    var driveSupplyCurrent = 0.0.amps
    var steeringStatorCurrent = 0.0.amps
    var steeringSupplyCurrent = 0.0.amps

    var drivePosition = 0.0.meters
    var steeringPosition = 0.0.degrees

    var driveVelocity = 0.0.meters.perSecond
    var steeringVelocity = 0.0.degrees.perSecond

    var driveTemp = 0.0.celsius
    var steeringTemp = 0.0.celsius

    var potentiometerOutputRaw = 0.0
    var potentiometerOutputRadians = 0.0.radians

    override fun toLog(table: LogTable?) {
      table?.put("driveAppliedVoltage", driveAppliedVoltage.inVolts)
      table?.put("steeringAppliedVoltage", steeringAppliedVoltage.inVolts)
      table?.put("driveStatorCurrentAmps", driveStatorCurrent.inAmperes)
      table?.put("driveSupplyCurrentAmps", driveSupplyCurrent.inAmperes)
      table?.put("steeringStatorCurrentAmps", steeringStatorCurrent.inAmperes)
      table?.put("steeringSupplyCurrentAmps", steeringSupplyCurrent.inAmperes)
      table?.put("drivePositionMeters", drivePosition.inMeters)
      table?.put("steeringPositionRadians", steeringPosition.inRadians)
      table?.put("driveVelocityMetersPerSecond", driveVelocity.inMetersPerSecond)
      table?.put("steeringVelocityRadiansPerSecond", steeringVelocity.inRadiansPerSecond)
      table?.put("driveTempCelcius", driveTemp.inCelsius)
      table?.put("steeringTempCelcius", steeringTemp.inCelsius)
      table?.put("potentiometerOutputRaw", potentiometerOutputRaw)
      table?.put("potentiometerOutputRadians", potentiometerOutputRadians.inRadians)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("driveAppliedVoltage", driveAppliedVoltage.inVolts)?.let {
        driveAppliedVoltage = it.volts
      }
      table?.getDouble("steeringAppliedVoltage", steeringAppliedVoltage.inVolts)?.let {
        steeringAppliedVoltage = it.volts
      }
      table?.getDouble("driveStatorCurrentAmps", driveStatorCurrent.inAmperes)?.let {
        driveStatorCurrent = it.amps
      }
      table?.getDouble("driveSupplyCurrentAmps", driveSupplyCurrent.inAmperes)?.let {
        driveSupplyCurrent = it.amps
      }
      table?.getDouble("steeringStatorCurrentAmps", steeringStatorCurrent.inAmperes)?.let {
        steeringStatorCurrent = it.amps
      }
      table?.getDouble("steeringSupplyCurrentAmps", steeringSupplyCurrent.inAmperes)?.let {
        steeringSupplyCurrent = it.amps
      }
      table?.getDouble("drivePositionMeters", drivePosition.inMeters)?.let {
        drivePosition = it.meters
      }
      table?.getDouble("steeringPositionRadians", steeringPosition.inRadians)?.let {
        steeringPosition = it.radians
      }
      table?.getDouble("driveVelocityMetersPerSecond", driveVelocity.inMetersPerSecond)?.let {
        driveVelocity = it.meters.perSecond
      }
      table?.getDouble("steeringVelocityRadiansPerSecond", steeringVelocity.inRadiansPerSecond)
        ?.let { steeringVelocity = it.radians.perSecond }
      table?.getDouble("driveTempCelcius", driveTemp.inCelsius)?.let { driveTemp = it.celsius }
      table?.getDouble("steeringTempCelcius", steeringTemp.inCelsius)?.let {
        steeringTemp = it.celsius
      }
      table?.getDouble("potentiometerOutputRaw", potentiometerOutputRaw)?.let {
        potentiometerOutputRaw = it
      }
      table?.getDouble("potentiometerOutputRaw", potentiometerOutputRadians.inRadians)?.let {
        potentiometerOutputRadians = it.radians
      }
    }
  }

  val label: String

  fun updateInputs(inputs: SwerveModuleIOInputs) {}

  fun setSteeringSetpoint(angle: Angle) {}
  fun setClosedLoop(steering: Angle, speed: LinearVelocity, acceleration: LinearAcceleration) {}
  fun setOpenLoop(steering: Angle, speed: Double) {}

  fun resetModuleZero() {}
  fun zeroSteering() {}
  fun zeroDrive() {}

  fun setBrakeMode(brake: Boolean) {}

  fun configureDrivePID(kP: Double, kI: Double, kD: Double) {}
  fun configureSteeringPID(kP: Double, kI: Double, kD: Double) {}
  fun configureSteeringMotionMagic(maxVel: AngularVelocity, maxAccel: AngularAcceleration) {}
}
