package com.team4099.robot2023.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegrees

object IntakeIONeo : IntakeIO {

  private val rollerMotor =
    CANSparkMax(Constants.Intake.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderArmMotor =
    CANSparkMax(Constants.Intake.LEADER_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderArmEncoder = leaderArmMotor.getEncoder()

  private val throughBoreEncoder = DutyCycleEncoder(Constants.Intake.REV_ENCODER_PORT)

  private val leaderPIDController: SparkMaxPIDController = leaderArmMotor.pidController

  init {
    leaderArmMotor.restoreFactoryDefaults()
    leaderArmMotor.clearFaults()
    leaderArmMotor.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    leaderArmMotor.inverted = IntakeConstants.LEFT_MOTOR_INVERTED

    // TODO (Relative to NEO rotations)
    val currentRelativePosition =
      (throughBoreEncoder.get() - IntakeConstants.INTAKE_ZERO) *
        IntakeConstants.INTAKE_SENSOR_GEAR_RATIO
    leaderArmEncoder.setPosition(currentRelativePosition)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {}
  override fun setRollerPower(outputPower: Double) {
    rollerMotor.set(outputPower)
  }
  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    leaderPIDController.setFF(feedforward.inVolts)
    leaderPIDController.setReference(
      (armPosition / 360.degrees) * IntakeConstants.INTAKE_ARM_GEAR_RATIO,
      CANSparkMax.ControlType.kPosition
    )
  }
  override fun setArmVoltage(voltage: ElectricalPotential) {
    leaderArmMotor.setVoltage(voltage.inVolts)
  }
  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: ProportionalGain<Radian, Volt>,
    kD: ProportionalGain<Radian, Volt>
  ) {
    leaderPIDController.setP(kP.inVoltsPerDegrees)
    leaderPIDController.setI(kI.inVoltsPerDegrees)
    leaderPIDController.setD(kD.inVoltsPerDegrees)
  }
  override fun zeroEncoder() {
    leaderArmEncoder.setPosition(0.0)

  }
}
