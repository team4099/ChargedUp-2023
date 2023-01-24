package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.math.MathUtil
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val rollerSparkMax =
    CANSparkMax(Constants.Manipulator.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val intakeSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax,
      ManipulatorConstants.ROLLER_GEAR_RATIO,
      Constants.Universal.VOLTAGE_COMPENSATION
    )
  private val armSparkMax =
    CANSparkMax(Constants.Manipulator.ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val armSensor =
    sparkMaxLinearMechanismSensor(
      armSparkMax,
      ManipulatorConstants.ARM_GEAR_RATIO,
      ManipulatorConstants.ARM_SPOOL_RADIUS * 2,
      Constants.Universal.VOLTAGE_COMPENSATION
    )
  private val armPIDController: SparkMaxPIDController = armSparkMax.pidController

  init {
    rollerSparkMax.restoreFactoryDefaults()
    rollerSparkMax.clearFaults()
    armSparkMax.restoreFactoryDefaults()
    armSparkMax.clearFaults()

    rollerSparkMax.enableVoltageCompensation(Constants.Universal.VOLTAGE_COMPENSATION.inVolts)
    rollerSparkMax.setSmartCurrentLimit(
      ManipulatorConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    rollerSparkMax.inverted = ManipulatorConstants.ROLLER_MOTOR_INVERTED
    rollerSparkMax.burnFlash()
    rollerSparkMax.openLoopRampRate = ManipulatorConstants.ROLLER_RAMP_RATE

    rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)

    armSparkMax.enableVoltageCompensation(Constants.Universal.VOLTAGE_COMPENSATION.inVolts)
    armSparkMax.setSmartCurrentLimit(
      ManipulatorConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    armSparkMax.inverted = ManipulatorConstants.ARM_MOTOR_INVERTED
    armSparkMax.burnFlash()
    armSparkMax.openLoopRampRate = ManipulatorConstants.ARM_RAMP_RATE

    armSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
  }

  override fun setRollerPower(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(MathUtil.clamp(voltage.inVolts, -Constants.Universal.VOLTAGE_COMPENSATION.inVolts, Constants.Universal.VOLTAGE_COMPENSATION.inVolts))
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerPosition = intakeSensor.position
    inputs.rollerVelocity = intakeSensor.velocity
    inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
    inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SuplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput
    inputs.rollerTemp = rollerSparkMax.motorTemperature.celsius

    inputs.armPosition = armSensor.position
    inputs.armVelocity = armSensor.velocity
    inputs.armAppliedVoltage = armSparkMax.busVoltage.volts * armSparkMax.appliedOutput
    inputs.armStatorCurrent = armSparkMax.outputCurrent.amps
    // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SuplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.armSupplyCurrent = inputs.armStatorCurrent * armSparkMax.appliedOutput
    inputs.armTemp = armSparkMax.motorTemperature.celsius
  }
  override fun setArmOpenLoop(percentOutput: Double) {
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

    armPIDController.p = armSensor.proportionalPositionGainToRawUnits(kP)
    armPIDController.i = armSensor.integralPositionGainToRawUnits(kI)
    armPIDController.d = armSensor.derivativePositionGainToRawUnits(kD)
  }

  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }
  }

  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      armSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      armSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }
  }
}
