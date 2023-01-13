package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.ManipulatorConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val intakeSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val intakeSensor =
    sparkMaxAngularMechanismSensor(
      intakeSparkMax,
      ManipulatorConstants.GEAR_RATIO,
      ManipulatorConstants.VOLTAGE_COMPENSATION
    )

  init {
    intakeSparkMax.restoreFactoryDefaults()
    intakeSparkMax.clearFaults()

    // TODO(check if this is right)
    intakeSparkMax.enableVoltageCompensation(ManipulatorConstants.VOLTAGE_COMPENSATION.inVolts)
    intakeSparkMax.setSmartCurrentLimit(ManipulatorConstants.SUPPLY_CURRENT_LIMIT)
    intakeSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    intakeSparkMax.inverted = ManipulatorConstants.INTAKE_MOTOR_INVERTED
    intakeSparkMax.burnFlash()
    intakeSparkMax.setOpenLoopRampRate(ManipulatorConstants.RAMP_RATE)
  }

  override fun setRollerPower(percentOutput: Double) {
    intakeSparkMax.set(percentOutput)
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerPosition = intakeSensor.position
    inputs.rollerVelocity = intakeSensor.velocity
    inputs.rollerStatorCurrent = intakeSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SuplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * intakeSparkMax.appliedOutput
    inputs.rollerTempCelcius = intakeSparkMax.motorTemperature.celsius
  }
}
