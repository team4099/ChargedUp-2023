package com.team4099.robot2023.subsystems.elevator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inOutputPerSecond
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object ElevatorIONeo : ElevatorIO {

  private val leaderSparkMax =
    CANSparkMax(Constants.Elevator.LEADER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderSensor =
    sparkMaxLinearMechanismSensor(
      leaderSparkMax,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_RADIUS * 2,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSparkMax =
    CANSparkMax(Constants.Elevator.FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val followerSensor =
    sparkMaxLinearMechanismSensor(
      followerSparkMax,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_RADIUS * 2,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val leaderPIDController: SparkMaxPIDController = leaderSparkMax.pidController
  private val followerPIDController: SparkMaxPIDController = followerSparkMax.pidController

  init {
    leaderSparkMax.restoreFactoryDefaults()
    followerSparkMax.restoreFactoryDefaults()

    leaderSparkMax.clearFaults()
    followerSparkMax.clearFaults()

    leaderSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)
    followerSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)

    leaderSparkMax.inverted = ElevatorConstants.LEFT_MOTOR_INVERTED
    followerSparkMax.inverted = ElevatorConstants.RIGHT_MOTOR_INVERTED

    leaderSparkMax.setSmartCurrentLimit(ElevatorConstants.PHASE_CURRENT_LIMIT.inAmperes.toInt())
    followerSparkMax.setSmartCurrentLimit(ElevatorConstants.PHASE_CURRENT_LIMIT.inAmperes.toInt())

    leaderSparkMax.openLoopRampRate = ElevatorConstants.RAMP_RATE.inOutputPerSecond
    followerSparkMax.openLoopRampRate = ElevatorConstants.RAMP_RATE.inOutputPerSecond

    followerSparkMax.follow(leaderSparkMax)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    // TODO(is there any reason to log follower aswell)
    inputs.elevatorPosition = leaderSensor.position

    inputs.elevatorVelocity = leaderSensor.velocity

    // voltage in * percent out
    inputs.leaderAppliedVoltage = leaderSparkMax.busVoltage.volts * leaderSparkMax.appliedOutput

    inputs.leaderStatorCurrent = leaderSparkMax.outputCurrent.amps

    // BatteryVoltage * SupplyCurrent = percentOutput * BatteryVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent

    inputs.leaderSupplyCurrent = inputs.leaderStatorCurrent * leaderSparkMax.appliedOutput

    inputs.leaderTempCelcius = leaderSparkMax.motorTemperature.celsius

    // voltage in * percent out
    inputs.followerAppliedVoltage = leaderSparkMax.busVoltage.volts * followerSparkMax.appliedOutput

    inputs.followerStatorCurrent = followerSparkMax.outputCurrent.amps

    inputs.followerSupplyCurrent = inputs.followerStatorCurrent * followerSparkMax.appliedOutput

    inputs.followerTempCelcius = followerSparkMax.motorTemperature.celsius
  }

  override fun setOutputVoltage(voltage: ElectricalPotential) {
    leaderSparkMax.setVoltage(voltage.inVolts)
  }

  override fun setPosition(position: Length, feedforward: ElectricalPotential) {

    leaderPIDController.setFF(feedforward.inVolts)
    followerPIDController.setFF(feedforward.inVolts)

    leaderPIDController.setReference(position.inMeters, CANSparkMax.ControlType.kPosition)
  }

  override fun zeroEncoder() {
    leaderSparkMax.encoder.position = 0.0
    followerSparkMax.encoder.position = 0.0
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    leaderPIDController.p = leaderSensor.proportionalPositionGainToRawUnits(kP)
    leaderPIDController.i = leaderSensor.integralPositionGainToRawUnits(kI)
    leaderPIDController.d = leaderSensor.derivativePositionGainToRawUnits(kD)

    followerPIDController.p = followerSensor.proportionalPositionGainToRawUnits(kP)
    followerPIDController.i = followerSensor.integralPositionGainToRawUnits(kI)
    followerPIDController.d = followerSensor.derivativePositionGainToRawUnits(kD)
  }
}
