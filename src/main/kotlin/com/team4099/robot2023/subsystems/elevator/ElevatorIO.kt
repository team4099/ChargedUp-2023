package com.team4099.robot2023.subsystems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond


interface ElevatorIO {
  class ElevatorInputs : LoggableInputs {

    var elevatorPosition = 0.0.inches
    var elevatorVelocity = 0.0.inches.perSecond

    var leaderAppliedVoltage = 0.volts
    var followerAppliedVoltage = 0.volts

    var leaderSupplyCurrent = 0.0.amps
    var leaderStatorCurrent = 0.0.amps

    var followerSupplyCurrent = 0.0.amps
    var followerStatorCurrent = 0.0.amps

    var leaderTempCelcius = 0.0.celsius
    var followerTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable) {
      table?.put("elevatorPositionRad", elevatorPosition.inInches)
      table?.put("elevatorVelocityRadPerSec", elevatorVelocity.inInchesPerSecond)
      table?.put("elevatorAppliedVolts", leaderAppliedVoltage.inVolts)
      table?.put("elevatorAppliedVolts", followerAppliedVoltage.inVolts)
      table?.put("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
      table?.put("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)
      table?.put("elevatorLeaderCurrentAmps", leaderSupplyCurrent.inAmperes)
      table?.put("elevatorFollowerCurrentAmps", followerSupplyCurrent.inAmperes)
      table?.put("elevatorLeaderTempCelcius", leaderTempCelcius.inCelsius)
      table?.put("elevatorFollowerTempCelcius", followerTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable) {
<<<<<<< HEAD
      TODO("Not yet implemented")

=======
      table?.getDouble("elevatorPositionRad", elevatorPosition.inInches)?.let {
        elevatorPosition = it.inches
      }
      table?.getDouble("elevatorVelocityRadPerSec", elevatorVelocity.inInchesPerSecond)?.let {
        elevatorVelocity = it.inches.perSecond
      }
      table?.getDouble("elevatorLeaderAppliedVolts", leaderAppliedVoltage.inVolts)?.let {
        leaderAppliedVoltage = it.volts
      }
      table?.getDouble("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.getDouble("elevatorLeaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.getDouble("elevatorLeadertempCelcius", leaderTempCelcius.inCelsius)?.let {
        leaderTempCelcius = it.celsius
      }

      table?.getDouble("elevatorFollowerAppliedVolts", followerAppliedVoltage.inVolts)?.let {
        followerAppliedVoltage = it.volts
      }
      table?.getDouble("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)?.let {
        followerStatorCurrent = it.amps
      }
      table?.getDouble("elevatorFollowerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)?.let {
        followerSupplyCurrent = it.amps
      }
      table?.getDouble("elevatorFollowertempCelcius", followerTempCelcius.inCelsius)?.let {
        followerTempCelcius = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ElevatorInputs) {}

  fun setOpenLoop(percentOutput: Double) {}

  fun setPosition(height: Length) {}

  fun configPID(kP: Double, kI: Double, kD: Double) {}

}
