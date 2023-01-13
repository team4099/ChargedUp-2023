package com.team4099.robot2023.subsystems.elevator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.ElevatorConstants
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.sparkMaxLinearMechanismSensor
import kotlin.math.sin

object ElevatorIONeo : ElevatorIO {

  // TODO(update motor ID's)
  private val leaderSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderSensor =
    sparkMaxLinearMechanismSensor(
      leaderSparkMax,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_RADIUS * 2,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSparkMax = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val followerSensor =
    sparkMaxLinearMechanismSensor(
      followerSparkMax,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_RADIUS * 2,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  val leaderPIDController = leaderSparkMax.pidController
  val followerPIDController = followerSparkMax.pidController

  init {
    leaderSparkMax.restoreFactoryDefaults()
    followerSparkMax.restoreFactoryDefaults()

    leaderSparkMax.clearFaults()
    followerSparkMax.clearFaults()

    leaderSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)
    followerSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)

    leaderSparkMax.inverted = ElevatorConstants.LEFT_MOTOR_INVERTED
    followerSparkMax.inverted = ElevatorConstants.RIGHT_MOTOR_INVERTED

    leaderSparkMax.setSmartCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT)
    followerSparkMax.setSmartCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT)

    // TODO(figure out if we need this)
    leaderSparkMax.setOpenLoopRampRate(ElevatorConstants.RAMP_RATE)
    followerSparkMax.setOpenLoopRampRate(ElevatorConstants.RAMP_RATE)

    followerSparkMax.follow(leaderSparkMax)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    // TODO(is there any reason to log follower aswell)
    inputs.elevatorPosition = leaderSensor.position

    inputs.elevatorVelocity = leaderSensor.velocity

    inputs.leaderAppliedOutput = leaderSparkMax.appliedOutput

    inputs.leaderStatorCurrent = leaderSparkMax.outputCurrent.amps

    // BatteryVoltage * SupplyCurrent = percentOutput * BatteryVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BatteryVoltage
    // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
    // percentOutput * statorCurrent

    inputs.leaderSupplyCurrent = inputs.leaderStatorCurrent * leaderSparkMax.appliedOutput

    inputs.leaderTempCelcius = leaderSparkMax.motorTemperature.celsius

    inputs.followerAppliedOutput = followerSparkMax.appliedOutput

    inputs.followerStatorCurrent = followerSparkMax.outputCurrent.amps

    inputs.followerSupplyCurrent = inputs.followerStatorCurrent * followerSparkMax.appliedOutput

    inputs.followerTempCelcius = followerSparkMax.motorTemperature.celsius
  }

  override fun setOpenLoop(percentOutput: Double) {
    leaderSparkMax.set(percentOutput)
  }

  override fun setPosition(height: Length, feedforward: ElectricalPotential) {

    leaderPIDController.setFF(feedforward.inVolts)
    followerPIDController.setFF(feedforward.inVolts)

    val position = height / sin(ElevatorConstants.ELEVATOR_ANGLE.inRadians)

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

    leaderPIDController.setP(kP.inVoltsPerMeter)
    leaderPIDController.setI(kI.inVoltsPerMeterSeconds)
    leaderPIDController.setD(kD.inVoltsPerMeterPerSecond)

    followerPIDController.setP(kP.inVoltsPerMeter)
    followerPIDController.setI(kI.inVoltsPerMeterSeconds)
    followerPIDController.setD(kD.inVoltsPerMeterPerSecond)
  }
}
