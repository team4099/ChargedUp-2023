package com.team4099.robot2023.subsystems.motorchecker

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.REVLibError
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.Temperature
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

interface MotorType

typealias REVNeo = MotorType

typealias Falcon = MotorType

enum class CURRENT_STAGE_LIMIT {
  NONE,
  BASE,
  FIRST,
  SHUTDOWN
}

abstract class Motor<M : MotorType> {
  open val name: String = ""

  open val appliedVoltage: ElectricalPotential = 0.0.volts
  open val busVoltage: ElectricalPotential = 0.0.volts

  open val temperature: Temperature = 0.0.celsius

  open val statorCurrent: Current = 0.0.amps
  open val supplyCurrent: Current = 0.0.amps

  open var currentLimitStage: CURRENT_STAGE_LIMIT = CURRENT_STAGE_LIMIT.NONE

  open val baseCurrentLimit: Current = 0.0.amps

  open val firstStageTemperatureLimit: Temperature = 0.0.celsius
  open val firstStageCurrentLimit: Current = 0.0.amps

  open val motorShutDownThreshold: Temperature = 0.0.celsius

  open val currentLimitInUse: Current
    get() =
      if (currentLimitStage == CURRENT_STAGE_LIMIT.BASE) baseCurrentLimit
      else firstStageCurrentLimit

  open val id: Int = -1
  open fun setCurrentLimit(
    limit: Current,
    thresholdLimit: Current? = null,
    thresholdTime: Time? = null
  ): Boolean {
    return false
  }

  open fun shutdown(): Boolean {
    return false
  }
  //  inner class MotorLoggableInputs: LoggableInputs{
  //    override fun toLog(table: LogTable?) {
  //      table?.put("motorName", name)
  //      table?.put("${name}AppliedVoltageVolts", appliedVoltage.inVolts)
  //      table?.put("${name}BusVoltageVolts", busVoltage.inVolts)
  //      table?.put("${name}TemperatureCelsius", temperature.inCelsius)
  //      table?.put("${name}StatorCurrentAmps", statorCurrent.inAmperes)
  //      table?.put("${name}SupplyCurrentAmps", supplyCurrent.inAmperes)
  //      table?.put("${name}CurrentLimitStage", currentLimitStage.name)
  //      table?.put("${name}BaseCurrentLimitAmps", baseCurrentLimit.inAmperes)
  //      table?.put("${name}FirstStageTemperatureLimitCelsius",
  // firstStageTemperatureLimit.inCelsius)
  //      table?.put("${name}FirstStageCurrentLimitAmps", firstStageCurrentLimit.inAmperes)
  //      table?.put("${name}MotorShutDownThresholdCelsius", motorShutDownThreshold.inCelsius)
  //      table?.put("${name}CurrentLimitInUseAmps", currentLimitInUse.inAmperes)
  //      table?.put("${name}MotorID", id.toLong())
  //    }
  //
  //    override fun fromLog(table: LogTable?) {
  //      TODO("Not yet implemented") // don't tbh
  //    }
  //
  //  }
}

class Neo(
  private val canSparkMax: CANSparkMax,
  override val name: String,
  override val baseCurrentLimit: Current,
  override val firstStageTemperatureLimit: Temperature,
  override val firstStageCurrentLimit: Current,
  override val motorShutDownThreshold: Temperature
) : Motor<REVNeo>() {
  override val busVoltage: ElectricalPotential
    get() = canSparkMax.busVoltage.volts

  override val appliedVoltage: ElectricalPotential
    get() = busVoltage * canSparkMax.appliedOutput
  override val statorCurrent: Current
    get() = canSparkMax.outputCurrent.amps

  override val temperature: Temperature
    get() = canSparkMax.motorTemperature.celsius

  override val supplyCurrent: Current
    get() = statorCurrent * canSparkMax.appliedOutput

  override var currentLimitStage = CURRENT_STAGE_LIMIT.NONE
  override val id: Int
    get() = canSparkMax.deviceId

  override fun setCurrentLimit(
    limit: Current,
    thresholdLimit: Current?,
    thresholdTime: Time?
  ): Boolean {
    val statusCode = canSparkMax.setSmartCurrentLimit(limit.inAmperes.toInt())

    var secondaryLimitStatusCode: REVLibError = REVLibError.kError
    if (statusCode == REVLibError.kOk) {
      // setting secondary limit to something absolutely absurd so it'll never shut down because of
      // this
      secondaryLimitStatusCode =
        canSparkMax.setSecondaryCurrentLimit(currentLimitInUse.inAmperes + 150)

      if (limit == firstStageCurrentLimit) {
        currentLimitStage = CURRENT_STAGE_LIMIT.FIRST
      } else if (limit == baseCurrentLimit) {
        currentLimitStage = CURRENT_STAGE_LIMIT.BASE
      }
    }

    return statusCode == REVLibError.kOk && secondaryLimitStatusCode == REVLibError.kOk
  }

  override fun shutdown(): Boolean {
    // setting the stator current limit to 0, so it immediately triggers and shuts off motor output
    val statusCode = canSparkMax.setSecondaryCurrentLimit(0.0)
    if (statusCode == REVLibError.kOk) {
      currentLimitStage = CURRENT_STAGE_LIMIT.SHUTDOWN
    }

    return statusCode == REVLibError.kOk
  }
}

class Falcon500(
  private val falcon500: TalonFX,
  override val name: String,
  override val baseCurrentLimit: Current,
  override val firstStageTemperatureLimit: Temperature,
  override val firstStageCurrentLimit: Current,
  override val motorShutDownThreshold: Temperature = 110.celsius
) : Motor<Falcon>() {

  override val appliedVoltage: ElectricalPotential
    get() = falcon500.motorOutputVoltage.volts

  override val busVoltage: ElectricalPotential
    get() = falcon500.busVoltage.volts

  override val temperature: Temperature
    get() = falcon500.temperature.celsius

  override val statorCurrent: Current
    get() = falcon500.statorCurrent.amps

  override val supplyCurrent: Current
    get() = falcon500.supplyCurrent.amps

  override val id: Int
    get() = falcon500.deviceID

  override var currentLimitStage = CURRENT_STAGE_LIMIT.NONE

  override fun setCurrentLimit(
    limit: Current,
    thresholdLimit: Current?,
    thresholdTime: Time?
  ): Boolean {
    if (thresholdLimit != null && thresholdTime != null) {
      val statorCurrentConfigSuccess =
        falcon500.configStatorCurrentLimit(
          StatorCurrentLimitConfiguration(
            true, limit.inAmperes, thresholdLimit.inAmperes, thresholdTime.inSeconds
          )
        )

      if (statorCurrentConfigSuccess == ErrorCode.OK) {
        if (limit == firstStageCurrentLimit) {
          currentLimitStage = CURRENT_STAGE_LIMIT.FIRST
        } else if (limit == baseCurrentLimit) {
          currentLimitStage = CURRENT_STAGE_LIMIT.BASE
        }
      }

      return statorCurrentConfigSuccess == ErrorCode.OK
    }
    return false
  }

  override fun shutdown(): Boolean {
    return temperature > 110.celsius // falcons have in built temperature shut down
  }
}
