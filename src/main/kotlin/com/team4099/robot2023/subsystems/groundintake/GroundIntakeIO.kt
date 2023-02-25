package com.team4099.robot2023.subsystems.groundintake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface GroundIntakeIO {
  class GroundIntakeIOInputs : LoggableInputs {
    var armPosition: Angle = 0.0.degrees
    var armVelocity = 0.0.degrees.perSecond

    var armAbsoluteEncoderPosition = 0.0.degrees

    var armAppliedVoltage = 0.0.volts
    var armSupplyCurrent = 0.0.amps
    var armStatorCurrent = 0.0.amps
    var armTemp = 0.0.celsius

    var rollerVelocity = 0.0.rotations.perMinute

    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    var isSimulated = false

    override fun toLog(table: LogTable?) {
      table?.put("armPositionDegrees", armPosition.inDegrees)
      table?.put("armAbsoluteEncoderPositionDegrees", armAbsoluteEncoderPosition.inDegrees)
      table?.put("armVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)

      table?.put("armAppliedVoltage", armAppliedVoltage.inVolts)

      table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)

      table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)

      table?.put("armTempCelsius", armTemp.inCelsius)

      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)

      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)

      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)

      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("rollerTempCelsius", rollerTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("armPositionDegrees", armPosition.inDegrees)?.let {
        armPosition = it.degrees
      }
      table?.getDouble("armAbsoluteEncoderPositionDegrees", armAbsoluteEncoderPosition.inDegrees)?.let {
        armAbsoluteEncoderPosition = it.degrees
      }
      table?.getDouble("armVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)?.let {
        armVelocity = it.degrees.perSecond
      }
      table?.getDouble("armAppliedVoltage", armAppliedVoltage.inVolts)?.let {
        armAppliedVoltage = it.volts
      }
      table?.getDouble("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
        armSupplyCurrent = it.amps
      }
      table?.getDouble("armStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
        armStatorCurrent = it.amps
      }
      table?.getDouble("armTempCelsius", armTemp.inCelsius)?.let { armTemp = it.celsius }

      table?.getDouble("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perSecond
      }
      table?.getDouble("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.getDouble("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: GroundIntakeIOInputs) {}

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setRollerVoltage(voltage: ElectricalPotential) {}

  /**
   * Sets the position of the arm motor, specifically the length of the arm
   *
   * @param armPosition the position to set the arm to
   * @param feedforward changes voltages to compensate for external forces
   */
  fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {}

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  fun setArmVoltage(voltage: ElectricalPotential) {}

  /**
   * Updates the PID constants using the implementation controller
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  /** Sets the current encoder position to be the zero value */
  fun zeroEncoder() {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setRollerBrakeMode(brake: Boolean) {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setArmBrakeMode(brake: Boolean) {}
}
