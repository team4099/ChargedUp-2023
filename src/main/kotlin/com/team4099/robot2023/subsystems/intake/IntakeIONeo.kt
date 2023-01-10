package com.team4099.robot2023.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.IntakeConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor

object IntakeIONeo : IntakeIO {
  private val intakeSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val intakeSensor =
    sparkMaxAngularMechanismSensor(
      intakeSparkMax, IntakeConstants.GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION
    )

  init {
    intakeSparkMax.restoreFactoryDefaults()
    intakeSparkMax.clearFaults()

    // TODO(check if this is right)
    intakeSparkMax.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    intakeSparkMax.setSmartCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT)
    intakeSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    intakeSparkMax.inverted = IntakeConstants.INTAKE_MOTOR_INVERTED
    intakeSparkMax.burnFlash()
    intakeSparkMax.setOpenLoopRampRate(IntakeConstants.RAMP_RATE)
  }

  override fun setGrabberPower(percentOutput: Double) {
    intakeSparkMax.set(percentOutput)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    inputs.grabberPosition = intakeSensor.position
    inputs.grabberVelocity = intakeSensor.velocity
    inputs.grabberStatorCurrent = intakeSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SuplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.grabberSupplyCurrent = inputs.grabberStatorCurrent * intakeSparkMax.appliedOutput
    inputs.grabberTempCelcius = intakeSparkMax.motorTemperature.celsius
  }
}
