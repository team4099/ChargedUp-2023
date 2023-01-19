package com.team4099.robot2023.subsystems.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var leaderArmPosition: Angle = 0.0.degrees
    var leaderArmVelocity = 0.0.degrees.perSecond

    var leaderArmAppliedOutput = 0.0
    var followerArmAppliedOutput = 0.0

    var leaderArmSupplyCurrent = 0.0.amps
    var followerArmSupplyCurrent = 0.0.amps

    var leaderArmStatorCurrent = 0.0.amps
    var followerArmStatorCurrent = 0.0.amps

    var leaderTempCelcius = 0.0.celsius
    var followerTempCelcius = 0.0.celsius

    var rollerApplied = 0.0

    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps

    var rollerTempCelcius = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("groundIntakePositionDegrees", leaderArmPosition.inDegrees)
      table?.put("groundIntakeVelocityDegreesPerSec", leaderArmVelocity.inDegreesPerSecond)

      table?.put("groundIntakeLeaderArmAppliedOutput", leaderArmAppliedOutput)
      table?.put("groundIntakeFollowerArmAppliedOutput", followerArmAppliedOutput)

      table?.put("groundIntakeLeaderArmSupplyCurrentAmps", leaderArmSupplyCurrent.inAmperes)
      table?.put("groundIntakeFollowerArmSupplyCurrentAmps", followerArmSupplyCurrent.inAmperes)

      table?.put("groundIntakeLeaderArmStatorCurrentAmps", leaderArmStatorCurrent.inAmperes)
      table?.put("groundIntakeFollowerArmStatorCurrentAmps", followerArmStatorCurrent.inAmperes)

      table?.put("groundIntakeLeaderTempCelcius", leaderTempCelcius.inCelsius)
      table?.put("groundIntakeFollowerTempCelcius", followerTempCelcius.inCelsius)

      table?.put("groundIntakeRollerApplied", rollerApplied)

      table?.put("groundIntakeRollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("groundIntakeRollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("groundIntakeRollerTempCelcius", rollerTempCelcius.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("groundIntakePositionDegrees", leaderArmPosition.inDegrees)?.let {
        leaderArmPosition = it.degrees
      }
      table?.getDouble("groundIntakeVelocityDegreesPerSec", leaderArmVelocity.inDegreesPerSecond)
        ?.let { leaderArmVelocity = it.degrees.perSecond }
      table?.getDouble("groundIntakeLeaderArmAppliedOutput", leaderArmAppliedOutput)?.let {
        leaderArmAppliedOutput = it
      }
      table?.getDouble("groundIntakeFollowerArmAppliedOutput", followerArmAppliedOutput)?.let {
        followerArmAppliedOutput = it
      }
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
      table?.getDouble("groundIntakeLeaderTempCelcius", leaderTempCelcius.inCelsius)?.let {
        leaderTempCelcius = it.celsius
      }
      table?.getDouble("groundIntakeFollowerTempCelcius", followerTempCelcius.inCelsius)?.let {
        followerTempCelcius = it.celsius
      }
      table?.getDouble("groundIntakeRollerApplied", rollerApplied)?.let { rollerApplied = it }
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

  fun setArmPosition(armPosition: Angle) {}
}
