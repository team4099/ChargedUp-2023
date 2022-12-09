package com.team4099.robot2022.subsystems.climber

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.inAmperes
import com.team4099.lib.units.base.inInches
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inInchesPerSecond
import com.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface TelescopingClimberIO {
  class TelescopingClimberIOInputs : LoggableInputs {
    var leftPosition = 0.0.meters
    var rightPosition = 0.0.meters

    var leftVelocity = 0.0.meters.perSecond
    var rightVelocity = 0.0.meters.perSecond

    var leftStatorCurrent = 0.0.amps
    var rightStatorCurrent = 0.0.amps

    var leftSupplyCurrent = 0.0.amps
    var rightSupplyCurrent = 0.0.amps

    var leftOutputVoltage = 0.0.volts
    var rightOutputVoltage = 0.0.volts

    var leftTemperatureCelsius = 0.0
    var rightTemperatureCelsius = 0.0

    override fun toLog(table: LogTable?) {
      table?.put("leftPositionInches", leftPosition.inInches)
      table?.put("rightPositionInches", rightPosition.inInches)
      table?.put("leftVelocityInchesPerSec", leftVelocity.inInchesPerSecond)
      table?.put("rightVelocityInchesPerSec", rightVelocity.inInchesPerSecond)
      table?.put("leftStatorCurrentAmps", leftStatorCurrent.inAmperes)
      table?.put("rightStatorCurrentAmps", rightStatorCurrent.inAmperes)
      table?.put("leftSupplyCurrentAmps", leftSupplyCurrent.inAmperes)
      table?.put("rightSupplyCurrentAmps", rightSupplyCurrent.inAmperes)
      table?.put("leftOutputVoltage", leftOutputVoltage.inVolts)
      table?.put("rightOutputVoltage", rightOutputVoltage.inVolts)
      table?.put("leftTemperatureCelsius", leftTemperatureCelsius)
      table?.put("rightTemperatureCelsius", rightTemperatureCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("leftPositionInches", leftPosition.inInches)?.let {
        leftPosition = it.inches
      }
      table?.getDouble("rightPositionInches", rightPosition.inInches)?.let {
        rightPosition = it.inches
      }
      table?.getDouble("leftVelocityInchesPerSec", leftVelocity.inInchesPerSecond)?.let {
        leftVelocity = it.inches.perSecond
      }
      table?.getDouble("rightVelocityInchesPerSec", rightVelocity.inInchesPerSecond)?.let {
        rightVelocity = it.inches.perSecond
      }
      table?.getDouble("leftStatorCurrentAmps", leftStatorCurrent.inAmperes)?.let {
        leftStatorCurrent = it.amps
      }
      table?.getDouble("rightStatorCurrentAmps", rightStatorCurrent.inAmperes)?.let {
        rightStatorCurrent = it.amps
      }
      table?.getDouble("leftSupplyCurrentAmps", leftSupplyCurrent.inAmperes)?.let {
        leftSupplyCurrent = it.amps
      }
      table?.getDouble("rightSupplyCurrentAmps", rightSupplyCurrent.inAmperes)?.let {
        rightSupplyCurrent = it.amps
      }
      table?.getDouble("leftOutputVoltage", leftOutputVoltage.inVolts)?.let {
        leftOutputVoltage = it.volts
      }
      table?.getDouble("rightOutputVoltage", rightOutputVoltage.inVolts)?.let {
        rightOutputVoltage = it.volts
      }
      table?.getDouble("leftTemperatureCelsius", leftTemperatureCelsius)?.let {
        leftTemperatureCelsius = it
      }
      table?.getDouble("rightTemperatureCelsius", leftTemperatureCelsius)?.let {
        rightTemperatureCelsius = it
      }
    }
  }

  fun updateInputs(inputs: TelescopingClimberIOInputs) {}

  fun setLeftOpenLoop(percentOutput: Double) {}
  fun setRightOpenLoop(percentOutput: Double) {}

  fun setLeftPosition(height: Length, feedforward: ElectricalPotential) {}
  fun setRightPosition(height: Length, feedforward: ElectricalPotential) {}

  fun zeroLeftEncoder() {}
  fun zeroRightEncoder() {}

  fun configPID(kP: Double, kI: Double, kD: Double) {}
}
