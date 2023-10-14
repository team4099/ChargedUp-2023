package com.team4099.robot2023.subsystems.falconspin

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.REVLibError
import edu.wpi.first.math.Num
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
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

typealias SimMotor = MotorType

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

  open var baseCurrentLimit: Current = 0.0.amps

  open var firstStageTemperatureLimit: Temperature = 0.0.celsius
  open var firstStageCurrentLimit: Current = 0.0.amps

  open var motorShutDownThreshold: Temperature = 0.0.celsius

  open val stickyFaults: List<String> = listOf()

  open val errors: List<String> = listOf()

  open val warnings: List<String> = listOf()

  open val info: List<String> = listOf()

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
  override var firstStageTemperatureLimit: Temperature = 70.celsius,
  override var motorShutDownThreshold: Temperature = 90.celsius
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

  override val stickyFaults: List<String>
    get() =
      canSparkMax
        .stickyFaults
        .toUInt()
        .toString(radix = 2)
        .mapIndexedNotNull { index, c -> index.takeIf { c == '1' } }
        .map { CANSparkMax.FaultID.fromId(it)?.name ?: "" }

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
) : Motor<Falcon>() {

  override val appliedVoltage: ElectricalPotential
    get() = (falcon500.get() * RobotController.getBatteryVoltage()).volts

  override val busVoltage: ElectricalPotential
    get() = falcon500.supplyVoltage.value.volts

  override val temperature: Temperature
    get() = falcon500.deviceTemp.value.celsius

  override val statorCurrent: Current
    get() = falcon500.statorCurrent.value.amps

  override val supplyCurrent: Current
    get() = falcon500.supplyCurrent.value.amps

  override val id: Int
    get() = falcon500.deviceID

  override var currentLimitStage = CURRENT_STAGE_LIMIT.NONE

  override val stickyFaults: List<String>
    get() {
      val retVal = mutableListOf<String>()

      if (falcon500.stickyFault_Hardware.value) {
        retVal.add("HardwareFault")
      }
      if (falcon500.stickyFault_ProcTemp.value) {
        retVal.add("ProcessorTemperature")
      }
      if (falcon500.stickyFault_DeviceTemp.value) {
        retVal.add("DeviceTemperature")
      }
      if (falcon500.stickyFault_Undervoltage.value) {
        retVal.add("UnderVoltage")
      }
      if (falcon500.stickyFault_BootDuringEnable.value) {
        retVal.add("DeviceBootDuringEnable")
      }
      if (falcon500.stickyFault_UnlicensedFeatureInUse.value) {
        retVal.add("Unlicensed")
      }
      if (falcon500.stickyFault_OverSupplyV.value) {
        retVal.add("OverSupplyVoltage")
      }
      if (falcon500.stickyFault_UnstableSupplyV.value) {
        retVal.add("UnstableSupplyVoltage")
      }
      if (falcon500.stickyFault_UnstableSupplyV.value) {
        retVal.add("UnstableSupplyVoltage")
      }
      if (falcon500.stickyFault_ForwardHardLimit.value) {
        retVal.add("ForwardLimitSwitch")
      }
      if (falcon500.stickyFault_ReverseHardLimit.value) {
        retVal.add("ReverseLimitSwitch")
      }
      if (falcon500.stickyFault_ForwardSoftLimit.value) {
        retVal.add("ForwardSoftLimit")
      }
      if (falcon500.stickyFault_ReverseSoftLimit.value) {
        retVal.add("ReverseSoftLimit")
      }
      if (falcon500.stickyFault_MissingRemoteSensor.value) {
        retVal.add("MissingRemoteSensor")
      }
      if (falcon500.stickyFault_FusedSensorOutOfSync.value) {
        retVal.add("MissingRemoteSensor")
      }
      if (falcon500.stickyFault_StatorCurrLimit.value) {
        retVal.add("StatorCurrentLimit")
      }
      if (falcon500.stickyFault_SupplyCurrLimit.value) {
        retVal.add("SupplyCurrentLimit")
      }
      if (falcon500.stickyFault_UsingFusedCANcoderWhileUnlicensed.value) {
        retVal.add("UsingFusedCANcoderWhileUnlicensed")
      }

      return retVal
    }

  override fun setCurrentLimit(
    limit: Current,
    thresholdLimit: Current?,
    thresholdTime: Time?
  ): Boolean {
    if (thresholdLimit != null && thresholdTime != null) {
      val supplyCurrentConfig = CurrentLimitsConfigs()
      supplyCurrentConfig.SupplyCurrentLimit = limit.inAmperes
      supplyCurrentConfig.SupplyCurrentThreshold = thresholdLimit.inAmperes
      supplyCurrentConfig.SupplyTimeThreshold = thresholdTime.inSeconds
      val supplyCurrentConfigSuccess = falcon500.configurator.apply(supplyCurrentConfig)

      if (supplyCurrentConfigSuccess == StatusCode.OK) {
        if (limit == firstStageCurrentLimit) {
          currentLimitStage = CURRENT_STAGE_LIMIT.FIRST
        } else if (limit == baseCurrentLimit) {
          currentLimitStage = CURRENT_STAGE_LIMIT.BASE
        }
      }

      return supplyCurrentConfigSuccess == StatusCode.OK
    }
    return false
  }

  override fun shutdown(): Boolean {
    return temperature > 110.celsius // falcons have in built temperature shut down
  }
}

class SimulatedMotor(
  val simSystem: LinearSystemSim<out Num, out Num, out Num>,
  override val name: String,
) : Motor<SimMotor>() {
  override val appliedVoltage: ElectricalPotential
    get() = simSystem.output.get(0, 0).volts

  override val statorCurrent: Current
    get() = simSystem.currentDrawAmps.amps

  override val busVoltage: ElectricalPotential
    get() = RoboRioSim.getVInVoltage().volts

  override val supplyCurrent: Current
    get() = statorCurrent * (appliedVoltage / busVoltage)

  override val stickyFaults: List<String>
    get() = listOf("")

  override var currentLimitStage = CURRENT_STAGE_LIMIT.NONE

  override fun shutdown(): Boolean {
    simSystem.setInput(0.0)
    return true
  }

  override fun setCurrentLimit(
    limit: Current,
    thresholdLimit: Current?,
    thresholdTime: Time?
  ): Boolean {
    if (limit == firstStageCurrentLimit) {
      currentLimitStage = CURRENT_STAGE_LIMIT.FIRST
    } else if (limit == baseCurrentLimit) {
      currentLimitStage = CURRENT_STAGE_LIMIT.BASE
    }

    return true
  }
}
