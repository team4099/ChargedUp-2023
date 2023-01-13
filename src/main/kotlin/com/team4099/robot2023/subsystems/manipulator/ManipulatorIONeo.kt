package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.ManipulatorConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val manipulatorSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val manipulatorSensor =
    sparkMaxAngularMechanismSensor(
      manipulatorSparkMax,
      ManipulatorConstants.GEAR_RATIO,
      ManipulatorConstants.VOLTAGE_COMPENSATION
    )

  init {
    manipulatorSparkMax.restoreFactoryDefaults()
    manipulatorSparkMax.clearFaults()

    // TODO(check if this is right)
    manipulatorSparkMax.enableVoltageCompensation(ManipulatorConstants.VOLTAGE_COMPENSATION.inVolts)
    manipulatorSparkMax.setSmartCurrentLimit(ManipulatorConstants.SUPPLY_CURRENT_LIMIT)
    manipulatorSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    manipulatorSparkMax.inverted = ManipulatorConstants.MOTOR_INVERTED
    manipulatorSparkMax.burnFlash()
    manipulatorSparkMax.setOpenLoopRampRate(ManipulatorConstants.RAMP_RATE)
  }

  override fun setRollerPower(percentOutput: Double) {
    manipulatorSparkMax.set(percentOutput)
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerPosition = manipulatorSensor.position
    inputs.rollerVelocity = manipulatorSensor.velocity
    inputs.rollerStatorCurrent = manipulatorSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SuplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * manipulatorSparkMax.appliedOutput
    inputs.rollerTempCelcius = manipulatorSparkMax.motorTemperature.celsius
  }
}
