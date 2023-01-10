package com.team4099.robot2023.subsystems.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {

    var grabberPosition = 0.degrees
    var grabberVelocity = 0.degrees.perSecond
    var grabberAppliedVoltage = 0.volts
    var grabberStatorCurrent = 0.amps
    var grabberSupplyCurrent = 0.amps
    var grabberTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable?) {
      // TODO: figure out why we did degrees and radians for this
      table?.put("grabberPositionRad", grabberPosition.inRadians)
      table?.put("grabberVelocityRadPerSec", grabberVelocity.inRadiansPerSecond)
      table?.put("grabberAppliedVolts", grabberAppliedVoltage.inVolts)
      table?.put("grabberStatorCurrentAmps", grabberStatorCurrent.inAmperes)
      table?.put("grabberSupplyCurrentAmps", grabberSupplyCurrent.inAmperes)
      table?.put("grabberTempCelcius", grabberTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable?) {

      table?.getDouble("grabberPositionRad", grabberPosition.inRadians)?.let {
        grabberPosition = it.radians
      }
      table?.getDouble("grabberVelocityRadPerSec", grabberVelocity.inRadiansPerSecond)?.let {
        grabberVelocity = it.radians.perSecond
      }
      table?.getDouble("grabberAppliedVolts", grabberAppliedVoltage.inVolts)?.let {
        grabberAppliedVoltage = it.volts
      }
      table?.getDouble("grabberStatorCurrentAmps", grabberStatorCurrent.inAmperes)?.let {
        grabberStatorCurrent = it.amps
      }
      table?.getDouble("grabberSupplyCurrentAmps", grabberSupplyCurrent.inAmperes)?.let {
        grabberSupplyCurrent = it.amps
      }
      table?.getDouble("grabberTempCelcius", grabberTempCelcius.inCelsius)?.let {
        grabberTempCelcius = it.celsius
      }
    }
  }

  fun updateInputs(inputs: IntakeIOInputs) {}

  fun setGrabberPower(percentOutPut: Double) {}
}
