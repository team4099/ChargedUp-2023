package com.team4099.robot2023.subsystems.groundintake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inPercentOutputPerSecond
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.IEEErem

object GroundIntakeIONeo : GroundIntakeIO {

  private val rollerSparkMax =
    CANSparkMax(Constants.Intake.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val armSparkMax =
    CANSparkMax(Constants.Intake.LEADER_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val rollerSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax,
      GroundIntakeConstants.ROLLER_GEAR_RATIO,
      GroundIntakeConstants.VOLTAGE_COMPENSATION
    )

  private val armSensor =
    sparkMaxAngularMechanismSensor(
      armSparkMax,
      GroundIntakeConstants.ROLLER_GEAR_RATIO,
      GroundIntakeConstants.VOLTAGE_COMPENSATION
    )

  private val armEncoder = armSparkMax.getEncoder()

  private val throughBoreEncoder = DutyCycleEncoder(Constants.Intake.REV_ENCODER_PORT)

  private val armPIDController: SparkMaxPIDController = armSparkMax.pidController

  val encoderAbsolutePosition: Angle
    get() {
      return (throughBoreEncoder.get().rotations * GroundIntakeConstants.ARM_ENCODER_GEAR_RATIO)
    }

  val armAbsolutePosition: Angle
    get() {
      return (encoderAbsolutePosition + GroundIntakeConstants.ABSOLUTE_ENCODER_OFFSET).inDegrees
        .IEEErem(360.0)
        .degrees
    }

  init {
    rollerSparkMax.restoreFactoryDefaults()
    rollerSparkMax.clearFaults()

    rollerSparkMax.enableVoltageCompensation(GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    rollerSparkMax.setSmartCurrentLimit(
      GroundIntakeConstants.ROLLER_CURRENT_LIMIT.inAmperes.toInt()
    )
    rollerSparkMax.inverted = GroundIntakeConstants.ROLLER_MOTOR_INVERTED
    rollerSparkMax.burnFlash()

    rollerSparkMax.openLoopRampRate =
      GroundIntakeConstants.ROLLER_RAMP_RATE.inPercentOutputPerSecond
    rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)

    armSparkMax.restoreFactoryDefaults()
    armSparkMax.clearFaults()

    armSparkMax.enableVoltageCompensation(GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    armSparkMax.setSmartCurrentLimit(GroundIntakeConstants.ARM_CURRENT_LIMIT.inAmperes.toInt())
    armSparkMax.inverted = GroundIntakeConstants.LEFT_MOTOR_INVERTED
    armSparkMax.burnFlash()

    armSparkMax.openLoopRampRate = GroundIntakeConstants.ROLLER_RAMP_RATE.inPercentOutputPerSecond
    armSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)

    zeroEncoder()
  }

  override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
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

    // same math as  rollersupplycurrent
    inputs.armSupplyCurrent = inputs.armStatorCurrent * armSparkMax.appliedOutput

    Logger.getInstance()
      .recordOutput("Intake/AbsoluteEncoderPosition", encoderAbsolutePosition.inDegrees)
    Logger.getInstance().recordOutput("Intake/AbsoluteArmPosition", armAbsolutePosition.inDegrees)
  }

  override fun setRollerPower(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      MathUtil.clamp(
        voltage.inVolts,
        -GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts,
        GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts
      )
    )
  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSparkMax.setVoltage(voltage.inVolts)
  }

  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    armPIDController.ff = feedforward.inVolts
    armPIDController.setReference(
      armSensor.positionToRawUnits(armPosition), CANSparkMax.ControlType.kPosition
    )
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armPIDController.p = armSensor.proportionalPositionGainToRawUnits(kP)
    armPIDController.i = armSensor.integralPositionGainToRawUnits(kI)
    armPIDController.d = armSensor.derivativePositionGainToRawUnits(kD)
  }

  override fun zeroEncoder() {
    armEncoder.setPosition(armSensor.positionToRawUnits(armAbsolutePosition))
  }

  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    }
  }

  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    } else {
      rollerSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
    }
  }
}
