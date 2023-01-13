package com.team4099.robot2023.subsystems.manipulator

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

interface ManipulatorIO {
  class ManipulatorIOInputs : LoggableInputs {

    var rollerPosition = 0.degrees
    var rollerVelocity = 0.degrees.perSecond
    var rollerAppliedVoltage = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable?) {
      // TODO: figure out why we did degrees and radians for this
      table?.put("rollerPositionRad", rollerPosition.inRadians)
      table?.put("rollerVelocityRadPerSec", rollerVelocity.inRadiansPerSecond)
      table?.put("rollerAppliedVolts", rollerAppliedVoltage.inVolts)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerTempCelcius", rollerTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable?) {

      table?.getDouble("rollerPositionRad", rollerPosition.inRadians)?.let {
        rollerPosition = it.radians
      }
      table?.getDouble("rollerVelocityRadPerSec", rollerVelocity.inRadiansPerSecond)?.let {
        rollerVelocity = it.radians.perSecond
      }
      table?.getDouble("rollerAppliedVolts", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.getDouble("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("rollerTempCelcius", rollerTempCelcius.inCelsius)?.let {
        rollerTempCelcius = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ManipulatorIOInputs) {}

  fun setRollerPower(percentOutPut: Double) {}
}
