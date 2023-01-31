package com.team4099.robot2023.subsystems.manipulator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface ManipulatorIO {
  class ManipulatorIOInputs : LoggableInputs {
    // TODO(Evaluate inputs. Pos, Vel may not be needed)
    var rollerPosition = 0.degrees
    var rollerVelocity = 0.degrees.perSecond
    var rollerAppliedVoltage = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTemp = 0.0.celsius

    var armPosition = 0.0.meters
    var armVelocity = 0.0.meters.perSecond
    var armAppliedVoltage = 0.volts
    var armStatorCurrent = 0.amps
    var armSupplyCurrent = 0.amps
    var armTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      // TODO: figure out why we did degrees and radians for this
      table?.put("rollerPositionRad", rollerPosition.inRadians)
      table?.put("rollerVelocityRadPerSec", rollerVelocity.inRadiansPerSecond)
      table?.put("rollerAppliedVolts", rollerAppliedVoltage.inVolts)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerTempCelsius", rollerTemp.inCelsius)

      table?.put("armPositionMeters", armPosition.inMeters)
      table?.put("armVelocityMetersPerSec", armVelocity.inMetersPerSecond)
      table?.put("armAppliedVolts", armAppliedVoltage.inVolts)
      table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)
      table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)
      table?.put("armTempCelsius", armTemp.inCelsius)
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
      table?.getDouble("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }

      table?.getDouble("armPositionMeters", armPosition.inMeters)?.let { armPosition = it.meters }
      table?.getDouble("armVelocityMetersPerSec", armVelocity.inMetersPerSecond)?.let {
        armVelocity = it.meters.perSecond
      }
      table?.getDouble("armAppliedVolts", armAppliedVoltage.inVolts)?.let {
        armAppliedVoltage = it.volts
      }
      table?.getDouble("armStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
        armStatorCurrent = it.amps
      }
      table?.getDouble("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
        armSupplyCurrent = it.amps
      }
      table?.getDouble("armTempCelsius", armTemp.inCelsius)?.let { armTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: ManipulatorIOInputs) {}

  fun setRollerPower(voltage: ElectricalPotential) {}

  fun setArmVoltage(voltage: ElectricalPotential) {}

  fun setPosition(position: Length, feedforward: ElectricalPotential) {}

  fun zeroEncoder() {}

  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}

  fun setRollerBrakeMode(brake: Boolean) {}

  fun setArmBrakeMode(brake: Boolean) {}
}
