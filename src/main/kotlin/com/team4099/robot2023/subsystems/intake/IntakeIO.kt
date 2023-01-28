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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var armPosition: Angle = 0.0.degrees
    var armVelocity = 0.0.degrees.perSecond

    var leaderArmAppliedVoltage = 0.0.volts
    var followerArmAppliedVoltage = 0.0.volts

    var leaderArmSupplyCurrent = 0.0.amps
    var followerArmSupplyCurrent = 0.0.amps

    var leaderArmStatorCurrent = 0.0.amps
    var followerArmStatorCurrent = 0.0.amps

    var leaderTemp = 0.0.celsius
    var followerTemp = 0.0.celsius

    var rollerAppliedVoltage = 0.0.volts

    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps

    var rollerTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("groundIntakePositionDegrees", armPosition.inDegrees)
      table?.put("groundIntakeVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)

      table?.put("groundIntakeleaderarmAppliedVoltage", leaderArmAppliedVoltage.inVolts)
      table?.put("groundIntakefollowerArmAppliedVoltage", followerArmAppliedVoltage.inVolts)

      table?.put("groundIntakeLeaderArmSupplyCurrentAmps", leaderArmSupplyCurrent.inAmperes)
      table?.put("groundIntakeFollowerArmSupplyCurrentAmps", followerArmSupplyCurrent.inAmperes)

      table?.put("groundIntakeLeaderArmStatorCurrentAmps", leaderArmStatorCurrent.inAmperes)
      table?.put("groundIntakeFollowerArmStatorCurrentAmps", followerArmStatorCurrent.inAmperes)

      table?.put("groundIntakeLeaderTempCelcius", leaderTemp.inCelsius)
      table?.put("groundIntakeFollowerTempCelcius", followerTemp.inCelsius)

      table?.put("groundIntakeRollerAppliedVoltage", rollerAppliedVoltage.inVolts)

      table?.put("groundIntakeRollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("groundIntakeRollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("groundIntakeRollerTempCelcius", rollerTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("groundIntakePositionDegrees", armPosition.inDegrees)?.let {
        armPosition = it.degrees
      }
      table?.getDouble("groundIntakeVelocityDegreesPerSec", armVelocity.inDegreesPerSecond)?.let {
        armVelocity = it.degrees.perSecond
      }
      table?.getDouble("groundIntakeleaderarmAppliedVoltage", leaderArmAppliedVoltage.inVolts)
        ?.let { leaderArmAppliedVoltage = it.volts }
      table?.getDouble("groundIntakefollowerArmAppliedVoltage", followerArmAppliedVoltage.inVolts)
        ?.let { followerArmAppliedVoltage = it.volts }
      table?.getDouble("groundIntakeLeaderArmSupplyCurrentAmps", leaderArmSupplyCurrent.inAmperes)
        ?.let { leaderArmSupplyCurrent = it.amps }
      table?.getDouble(
        "groundIntakeFollowerArmSupplyCurrentAmps", followerArmSupplyCurrent.inAmperes
      )
        ?.let { followerArmSupplyCurrent = it.amps }
      table?.getDouble("groundIntakeLeaderArmStatorCurrentAmps", leaderArmStatorCurrent.inAmperes)
        ?.let { leaderArmStatorCurrent = it.amps }
      table?.getDouble(
        "groundIntakeFollowerArmStatorCurrentAmps", followerArmStatorCurrent.inAmperes
      )
        ?.let { followerArmStatorCurrent = it.amps }
      table?.getDouble("groundIntakeLeaderTempCelcius", leaderTemp.inCelsius)?.let {
        leaderTemp = it.celsius
      }
      table?.getDouble("groundIntakeFollowerTempCelcius", followerTemp.inCelsius)?.let {
        followerTemp = it.celsius
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
      table?.getDouble("groundIntakeRollerTempCelcius", rollerTempCelcius.inCelsius)?.let {
        rollerStatorCurrent = it.amps
      }
    }
  }

  fun updateInputs(inputs: IntakeIOInputs) {}
  fun setRollerPower(outputPower: Double) {}
  fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {}
  fun setArmVoltage(voltage: ElectricalPotential) {}
  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}
  fun zeroEncoder() {}
}
