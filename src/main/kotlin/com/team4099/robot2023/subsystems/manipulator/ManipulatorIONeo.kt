package com.team4099.robot2023.subsystems.manipulator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
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
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object ManipulatorIONeo : ManipulatorIO {
  private val rollerSparkMax =
    CANSparkMax(Constants.Manipulator.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val rollerSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax,
      ManipulatorConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
      ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
    )
  private val armSparkMax =
    CANSparkMax(Constants.Manipulator.ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val armSensor =
    sparkMaxLinearMechanismSensor(
      armSparkMax,
      ManipulatorConstants.ARM_GEAR_RATIO.asDrivenOverDriving,
      ManipulatorConstants.ARM_SPOOL_RADIUS * 2,
      ManipulatorConstants.ARM_VOLTAGE_COMPENSATION
    )
  private val armPIDController: SparkMaxPIDController = armSparkMax.pidController

  init {
    // resetting the motors
    rollerSparkMax.restoreFactoryDefaults()
    rollerSparkMax.clearFaults()
    armSparkMax.restoreFactoryDefaults()
    armSparkMax.clearFaults()

    // set-up voltage and current limits
    rollerSparkMax.enableVoltageCompensation(
      ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION.inVolts
    )
    rollerSparkMax.setSmartCurrentLimit(
      ManipulatorConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    // configure default settings
    rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    rollerSparkMax.inverted = ManipulatorConstants.ROLLER_MOTOR_INVERTED
//    rollerSparkMax.openLoopRampRate = ManipulatorConstants.ROLLER_RAMP_RATE
    rollerSparkMax.burnFlash()

    // set-up voltage and current limits
    armSparkMax.enableVoltageCompensation(ManipulatorConstants.ARM_VOLTAGE_COMPENSATION.inVolts)
    armSparkMax.setSmartCurrentLimit(
      ManipulatorConstants.ARM_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    armSparkMax.inverted = ManipulatorConstants.ARM_MOTOR_INVERTED
    armSparkMax.openLoopRampRate = ManipulatorConstants.ARM_RAMP_RATE

    // set-up voltage and current limits
    armSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    armSparkMax.burnFlash()
  }

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit, uses clamp to insure voltage is between battery voltage compensation
   *
   * @param voltage the voltage to set the motor to
   */
  override fun setRollerPower(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION,
        ManipulatorConstants.ROLLER_VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  /**
   * Updates the values being logged using the actual sensor/motor readings
   *
   * @param inputs object of the Manipulator LoggableInputs
   */
  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerVelocity = rollerSensor.velocity
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

  /**
   * Sets the voltage of the arm motor but also checks to make sure the voltage doesn't exceed limit
   *
   * @param voltage the voltage to set the motor to
   */
  override fun setArmVoltage(voltage: ElectricalPotential) {
    // divide by 2 cause 12 volts is too fast
    armSparkMax.setVoltage(
      clamp(
        voltage,
        -ManipulatorConstants.ARM_VOLTAGE_COMPENSATION / 2,
        ManipulatorConstants.ARM_VOLTAGE_COMPENSATION / 2
      )
        .inVolts
    )
  }

  /**
   * Sets the position of the arm motor, specifically the length of the arm. Uses the sparkMax PID
   * controller
   *
   * @param position the position to set the arm to
   * @param feedforward changes voltages to compensate for external forces
   */
  override fun setArmPosition(position: Length, feedforward: ElectricalPotential) {
    armPIDController.setReference(
      armSensor.positionToRawUnits(
        clamp(
          position,
          ManipulatorConstants.ARM_SOFTLIMIT_RETRACTION,
          ManipulatorConstants.ARM_SOFTLIMIT_EXTENSION
        )
      ),
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.inVolts
    )
  }

  /** Sets the current encoder position to be the zero value */
  override fun zeroEncoder() {
    armSparkMax.encoder.position = 0.0
  }

  /**
   * Updates the PID constants using the implementation controller, uses arm sensor to convert from
   * PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {

    armPIDController.p = armSensor.proportionalPositionGainToRawUnits(kP)
    armPIDController.i = armSensor.integralPositionGainToRawUnits(kI)
    armPIDController.d = armSensor.derivativePositionGainToRawUnits(kD)
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }
  }

  /**
   * Sets the arm brake mode
   *
   * @param brake if it brakes
   */
  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      armSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      armSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }
  }
}
