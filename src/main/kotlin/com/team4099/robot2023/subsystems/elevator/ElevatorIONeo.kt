package com.team4099.robot2023.subsystems.elevator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inPercentOutputPerSecond
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.asDrivenOverDriving
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object ElevatorIONeo : ElevatorIO {

  private val leaderSparkMax =
    CANSparkMax(Constants.Elevator.LEADER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderSensor =
    sparkMaxLinearMechanismSensor(
      leaderSparkMax,
      ElevatorConstants.GEAR_RATIO.asDrivenOverDriving,
      ElevatorConstants.SPOOL_RADIUS * 2,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSparkMax =
    CANSparkMax(Constants.Elevator.FOLLOWER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderPIDController: SparkMaxPIDController = leaderSparkMax.pidController

  init {

    // reseting motor
    leaderSparkMax.restoreFactoryDefaults()
    followerSparkMax.restoreFactoryDefaults()

    leaderSparkMax.clearFaults()
    followerSparkMax.clearFaults()

    // basic settings
    leaderSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)
    followerSparkMax.enableVoltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION.inVolts)

    leaderSparkMax.inverted = true

    leaderSparkMax.setSmartCurrentLimit(ElevatorConstants.PHASE_CURRENT_LIMIT.inAmperes.toInt())
    followerSparkMax.setSmartCurrentLimit(ElevatorConstants.PHASE_CURRENT_LIMIT.inAmperes.toInt())

    leaderSparkMax.openLoopRampRate = ElevatorConstants.RAMP_RATE.inPercentOutputPerSecond
    followerSparkMax.openLoopRampRate = ElevatorConstants.RAMP_RATE.inPercentOutputPerSecond

    leaderSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    followerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake

    // makes follower motor output exact same power as leader
    followerSparkMax.follow(leaderSparkMax, ElevatorConstants.FOLLOW_MOTOR_INVERTED)

    leaderPIDController.ff = 0.0

    leaderSparkMax.burnFlash()
    followerSparkMax.burnFlash()

    MotorChecker.add(
      "Elevator",
      "Extension",
      MotorCollection(
        mutableListOf(
          Neo(leaderSparkMax, "Leader Extension Motor"),
          Neo(followerSparkMax, "Follower Extension Motor")
        ),
        ElevatorConstants.PHASE_CURRENT_LIMIT,
        30.celsius,
        ElevatorConstants.PHASE_CURRENT_LIMIT - 30.amps,
        90.celsius
      ),
    )
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
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

    inputs.leaderRawPosition = leaderSparkMax.encoder.position

    inputs.followerRawPosition = followerSparkMax.encoder.position

    Logger.getInstance()
      .recordOutput("Elevator/leaderRawRotations", leaderSparkMax.encoder.position)
  }

  /**
   * Sets the voltage of the elevator motors but also checks to make sure elevator doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  override fun setOutputVoltage(voltage: ElectricalPotential) {
    // divide by 2 cause full power elevator is scary
    leaderSparkMax.setVoltage(voltage.inVolts)
  }

  /**
   * Sets the voltage of the elevator motors but also checks to make sure elevator doesn't exceed
   * limit
   *
   * @param position the target position the PID controller will use
   * @param feedforward change in voltage to account for external forces on the system
   */
  override fun setPosition(position: Length, feedforward: ElectricalPotential) {
    leaderPIDController.setReference(
      leaderSensor.positionToRawUnits(
        clamp(
          position,
          ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION,
          ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION
        )
      ),
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.inVolts,
    )
  }

  /** set the current encoder position to be the encoders zero value */
  override fun zeroEncoder() {
    leaderSparkMax.encoder.position = 0.0
    followerSparkMax.encoder.position = 0.0
  }

  /**
   * updates the PID controller values using the sensor measurement for proportional intregral and
   * derivative gain multiplied by the 3 PID constants
   *
   * @param kP a constant which will be used to scale the proportion gain
   * @param kI a constant which will be used to scale the integral gain
   * @param kD a constant which will be used to scale the derivative gain
   */
  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    leaderPIDController.p = leaderSensor.proportionalPositionGainToRawUnits(kP)
    leaderPIDController.i = leaderSensor.integralPositionGainToRawUnits(kI)
    leaderPIDController.d = leaderSensor.derivativePositionGainToRawUnits(kD)
  }
}
