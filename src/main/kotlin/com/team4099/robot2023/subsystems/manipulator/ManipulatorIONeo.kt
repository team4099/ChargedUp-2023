package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val intakeSparkMax =
    CANSparkMax(Constants.Manipulator.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val intakeSensor =
    sparkMaxAngularMechanismSensor(
      intakeSparkMax,
      ManipulatorConstants.INTAKE_GEAR_RATIO,
      ManipulatorConstants.INTAKE_VOLTAGE_COMPENSATION
    )
  private val armSparkMax =
    CANSparkMax(Constants.Manipulator.ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val armSensor =
    sparkMaxLinearMechanismSensor(
      armSparkMax,
      ManipulatorConstants.ARM_GEAR_RATIO,
      ManipulatorConstants.ARM_SPOOL_RADIUS * 2,
      ManipulatorConstants.ARM_VOLTAGE_COMPENSATION
    )
  private val armPIDController: SparkMaxPIDController = armSparkMax.pidController

  init {
    intakeSparkMax.restoreFactoryDefaults()
    intakeSparkMax.clearFaults()
    armSparkMax.restoreFactoryDefaults()
    armSparkMax.clearFaults()

    // TODO(check if this is right)
    intakeSparkMax.enableVoltageCompensation(
      ManipulatorConstants.INTAKE_VOLTAGE_COMPENSATION.inVolts
    )
    intakeSparkMax.setSmartCurrentLimit(ManipulatorConstants.INTAKE_SUPPLY_CURRENT_LIMIT)
    intakeSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    intakeSparkMax.inverted = ManipulatorConstants.INTAKE_MOTOR_INVERTED
    intakeSparkMax.burnFlash()
    intakeSparkMax.openLoopRampRate = ManipulatorConstants.INTAKE_RAMP_RATE

    armSparkMax.enableVoltageCompensation(ManipulatorConstants.ARM_VOLTAGE_COMPENSATION.inVolts)
    armSparkMax.setSmartCurrentLimit(ManipulatorConstants.ARM_SUPPLY_CURRENT_LIMIT)
    armSparkMax.inverted = ManipulatorConstants.ARM_MOTOR_INVERTED
    armSparkMax.burnFlash()
    armSparkMax.openLoopRampRate = ManipulatorConstants.ARM_RAMP_RATE
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
  override fun setOpenLoop(percentOutput: Double) {
    armSparkMax.set(percentOutput)
  }

  override fun setPosition(position: Length, feedforward: ElectricalPotential) {
    armPIDController.setFF(feedforward.inVolts)
  }

  override fun zeroEncoder() {
    armSparkMax.encoder.position = 0.0
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    armPIDController.p = kP.inVoltsPerMeter
    armPIDController.i = kI.inVoltsPerMeterSeconds
    armPIDController.d = kD.inVoltsPerMeterPerSecond
  }
}
