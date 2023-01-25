package com.team4099.robot2023.subsystems.elevator

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
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond

interface ElevatorIO {
  class ElevatorInputs : LoggableInputs {

    var elevatorPosition = 0.0.inches
    var elevatorVelocity = 0.0.inches.perSecond

    var leaderAppliedOutput = 0.0
    var followerAppliedOutput = 0.0

    var leaderSupplyCurrent = 0.0.amps
    var leaderStatorCurrent = 0.0.amps

    var followerSupplyCurrent = 0.0.amps
    var followerStatorCurrent = 0.0.amps

    var leaderTempCelcius = 0.0.celsius
    var followerTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable) {
      table?.put("elevatorPositionInches", elevatorPosition.inInches)
      table?.put("elevatorVelocityInchesPerSec", elevatorVelocity.inInchesPerSecond)
      table?.put("elevatorAppliedVolts", leaderAppliedOutput)
      table?.put("elevatorAppliedVolts", followerAppliedOutput)
      table?.put("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
      table?.put("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)
      table?.put("elevatorLeaderCurrentAmps", leaderSupplyCurrent.inAmperes)
      table?.put("elevatorFollowerCurrentAmps", followerSupplyCurrent.inAmperes)
      table?.put("elevatorLeaderTempCelcius", leaderTempCelcius.inCelsius)
      table?.put("elevatorFollowerTempCelcius", followerTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table?.getDouble("elevatorPositionInches", elevatorPosition.inInches)?.let {
        elevatorPosition = it.inches
      }
      table?.getDouble("elevatorVelocityInchesPerSec", elevatorVelocity.inInchesPerSecond)?.let {
        elevatorVelocity = it.inches.perSecond
      }
      table?.getDouble("elevatorLeaderAppliedVolts", leaderAppliedOutput)?.let {
        leaderAppliedOutput = it
      }
      table?.getDouble("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.getDouble("elevatorLeaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.getDouble("elevatorLeaderTempCelcius", leaderTempCelcius.inCelsius)?.let {
        leaderTempCelcius = it.celsius
      }

      table?.getDouble("elevatorFollowerAppliedVolts", followerAppliedOutput)?.let {
        followerAppliedOutput = it
      }
      table?.getDouble("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)?.let {
        followerStatorCurrent = it.amps
      }
      table?.getDouble("elevatorFollowerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)?.let {
        followerSupplyCurrent = it.amps
      }
      table?.getDouble("elevatorFollowerTempCelcius", followerTempCelcius.inCelsius)?.let {
        followerTempCelcius = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ElevatorInputs) {}

  fun setOpenLoop(percentOutput: Double) {}

  fun setPosition(height: Length, feedForward: ElectricalPotential) {}

  fun zeroEncoder() {}

  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}
}
