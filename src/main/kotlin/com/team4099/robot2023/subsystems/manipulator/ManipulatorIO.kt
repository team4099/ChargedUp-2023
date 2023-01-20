package com.team4099.robot2023.subsystems.manipulator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface ManipulatorIO {
  class ManipulatorIOInputs : LoggableInputs {
    // TODO(Evaluate inputs. Pos, Vel may not be needed)
    var rollerPosition = 0.degrees
    var rollerVelocity = 0.degrees.perSecond
    var rollerAppliedOutput = 0.0
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTempCelcius = 0.0.celsius

    var armPosition = 0.degrees
    var armVelocity = 0.degrees.perSecond
    var armAppliedOutput = 0.0
    var armStatorCurrent = 0.amps
    var armSupplyCurrent = 0.amps
    var armTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable?) {
      // TODO: figure out why we did degrees and radians for this
      table?.put("rollerPositionRad", rollerPosition.inRadians)
      table?.put("rollerVelocityRadPerSec", rollerVelocity.inRadiansPerSecond)
      table?.put("rollerAppliedVolts", rollerAppliedOutput)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerTempCelcius", rollerTempCelcius.inCelsius)

      table?.put("armPositionRad", armPosition.inRadians)
      table?.put("armVelocityRadPerSec", armVelocity.inRadiansPerSecond)
      table?.put("armAppliedVolts", armAppliedOutput)
      table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)
      table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)
      table?.put("armTempCelcius", armTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable?) {

      table?.getDouble("rollerPositionRad", rollerPosition.inRadians)?.let {
        rollerPosition = it.radians
      }
      table?.getDouble("rollerVelocityRadPerSec", rollerVelocity.inRadiansPerSecond)?.let {
        rollerVelocity = it.radians.perSecond
      }
      table?.getDouble("rollerAppliedVolts", rollerAppliedOutput)?.let { rollerAppliedOutput = it }
      table?.getDouble("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("rollerTempCelcius", rollerTempCelcius.inCelsius)?.let {
        rollerTempCelcius = it.celsius
      }

      table?.getDouble("armPositionRad", armPosition.inRadians)?.let { armPosition = it.radians }
      table?.getDouble("armVelocityRadPerSec", armVelocity.inRadiansPerSecond)?.let {
        armVelocity = it.radians.perSecond
      }
      table?.getDouble("armAppliedVolts", armAppliedOutput)?.let { armAppliedOutput = it }
      table?.getDouble("armStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
        armStatorCurrent = it.amps
      }
      table?.getDouble("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
        armSupplyCurrent = it.amps
      }
      table?.getDouble("armTempCelcius", armTempCelcius.inCelsius)?.let {
        armTempCelcius = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ManipulatorIOInputs) {}

  fun setRollerPower(percentOutput: Double) {}

  fun setOpenLoop(percentOutput: Double) {}

  fun setPosition(position: Length, feedforward: ElectricalPotential) {}

  fun zeroEncoder() {}

  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}
}
