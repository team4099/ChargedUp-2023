package com.team4099.robot2023.subsystems.manipulator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface ManipulatorIO {
  class ManipulatorIOInputs : LoggableInputs {
    var rollerVelocity = 0.rotations.perMinute
    var rollerAppliedVoltage = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTemp = 0.0.celsius

    var armPosition = 0.0.inches
    var armVelocity = 0.0.inches.perSecond
    var armAppliedVoltage = 0.volts
    var armStatorCurrent = 0.amps
    var armSupplyCurrent = 0.amps
    var armTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      // TODO: figure out why we did degrees and radians for this
      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)
      table?.put("rollerAppliedVolts", rollerAppliedVoltage.inVolts)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerTempCelsius", rollerTemp.inCelsius)

      table?.put("armPositionInches", armPosition.inInches)
      table?.put("armVelocityInchesPerSec", armVelocity.inInchesPerSecond)
      table?.put("armAppliedVolts", armAppliedVoltage.inVolts)
      table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)
      table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)
      table?.put("armTempCelsius", armTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
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

      table?.getDouble("armPositionInches", armPosition.inInches)?.let { armPosition = it.inches }
      table?.getDouble("armVelocityInchesPerSec", armVelocity.inInchesPerSecond)?.let {
        armVelocity = it.inches.perSecond
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

  /**
   * Updates the values being logged using the actual sensor/motor readings
   *
   * @param inputs object of the Manipulator LoggableInputs
   */
  fun updateInputs(inputs: ManipulatorIOInputs) {}

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setRollerPower(voltage: ElectricalPotential) {}

  /**
   * Sets the voltage of the arm motor but also checks to make sure the voltage doesn't exceed limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setArmVoltage(voltage: ElectricalPotential) {}

  /**
   * Sets the position of the arm motor, specifically the length of the arm
   *
   * @param position the position to set the arm to
   * @param feedforward changes voltages to compensate for external forces
   */
  fun setArmPosition(position: Length, feedforward: ElectricalPotential) {}

  /** Sets the current encoder position to be the zero value */
  fun zeroEncoder() {}

  /**
   * Updates the PID constants using the implementation controller
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setRollerBrakeMode(brake: Boolean) {}

  /**
   * Sets the arm brake mode
   *
   * @param brake if it brakes
   */
  fun setArmBrakeMode(brake: Boolean) {}
}
