package com.team4099.robot2023.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.team4099.lib.units.derived.inVolts

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

    // TODO (Make Sure Encoder increases as arm extends)
    val currentRelativePosition =
      (throughBoreEncoder.get() - IntakeConstants.INTAKE_ZERO) *
        IntakeConstants.INTAKE_SENSOR_RATIO / IntakeConstants.ENCODER_COUNTS
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {}

  override fun setRollerPower(outputPower: Double) {}
}
