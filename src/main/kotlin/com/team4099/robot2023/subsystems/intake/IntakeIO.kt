package com.team4099.robot2023.subsystems.intake

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
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var armPosition: Angle = 0.0.degrees
    var armVelocity = 0.0.degrees.perSecond

    var armAppliedVoltage = 0.0.volts
    var armSupplyCurrent = 0.0.amps
    var armStatorCurrent = 0.0.amps
    var leaderTemp = 0.0.celsius

    var rollerVelocity = 0.0.rotations.perMinute

    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("groundIntakePositionDegrees", armPosition.inDegrees)
      table?.put("groundIntakeVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)

      table?.put("groundIntakeArmAppliedVoltage", armAppliedVoltage.inVolts)

      table?.put("groundIntakeArmSupplyCurrentAmps", armSupplyCurrent.inAmperes)

      table?.put("groundIntakeArmStatorCurrentAmps", armStatorCurrent.inAmperes)

      table?.put("groundIntakeLeaderTempCelcius", leaderTemp.inCelsius)

      table?.put("groundIntakeRollerAppliedVoltage", rollerAppliedVoltage.inVolts)

      table?.put("groundIntakeRollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)

      table?.put("groundIntakeRollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("groundIntakeRollerTempCelcius", rollerTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("groundIntakePositionDegrees", armPosition.inDegrees)?.let {
        armPosition = it.degrees
      }
      table?.getDouble("groundIntakeVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)?.let {
        armVelocity = it.degrees.perSecond
      }
      table?.getDouble("groundIntakeArmAppliedVoltage", armAppliedVoltage.inVolts)?.let {
        armAppliedVoltage = it.volts
      }
      table?.getDouble("groundIntakeArmSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
        armSupplyCurrent = it.amps
      }
      table?.getDouble("groundIntakeArmStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
        armStatorCurrent = it.amps
      }
      table?.getDouble("groundIntakeLeaderTempCelcius", leaderTemp.inCelsius)?.let {
        leaderTemp = it.celsius
      }
      table?.getDouble("groundIntakeRollerApplied", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.getDouble("groundIntakeRollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.getDouble("groundIntakeRollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.getDouble("groundIntakeRollerTempCelcius", rollerTemp.inCelsius)?.let {
        rollerStatorCurrent = it.amps
      }
    }
  }

  fun updateInputs(inputs: IntakeIOInputs) {}

  fun setRollerPower(voltage: ElectricalPotential) {}

  fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {}

  fun setArmVoltage(voltage: ElectricalPotential) {}

  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun zeroEncoder() {}

  fun setRollerBrakeMode(brake: Boolean) {}
  fun setArmBrakeMode(brake: Boolean) {}
}
