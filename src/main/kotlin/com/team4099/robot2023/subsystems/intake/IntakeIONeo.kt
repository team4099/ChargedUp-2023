package com.team4099.robot2023.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import org.team4099.lib.units.derived.inVolts

object IntakeIONeo : IntakeIO {

  private val rollerMotor =
    CANSparkMax(Constants.Intake.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderArmMotor =
    CANSparkMax(Constants.Intake.LEADER_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val followerArmMotor =
    CANSparkMax(Constants.Intake.FOLLOWER_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderArmEncoder = leaderArmMotor.getEncoder()
  private val followerArmEncoder = leaderArmMotor.getEncoder()

  private val throughBoreEncoder = Encoder(0, 1, true, CounterBase.EncodingType.k4X)

  // TODO (Make Sure Encoder increases as arm extends)
  val currentRelativePosition =
    (throughBoreEncoder.get() - IntakeConstants.INTAKE_ZERO) / IntakeConstants.ENCODER_COUNTS *
      IntakeConstants.INTAKE_SENSOR_RATIO

  private val leaderPIDController: SparkMaxPIDController = leaderArmMotor.pidController
  private val followerPIDController: SparkMaxPIDController = followerArmMotor.pidController

  init {
    leaderArmMotor.restoreFactoryDefaults()
    followerArmMotor.restoreFactoryDefaults()

    leaderArmMotor.clearFaults()
    followerArmMotor.clearFaults()

    leaderArmMotor.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    followerArmMotor.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION.inVolts)

    leaderArmMotor.inverted = IntakeConstants.LEFT_MOTOR_INVERTED
    followerArmMotor.inverted = IntakeConstants.RIGHT_MOTOR_INVERTED

    followerArmMotor.follow(leaderArmMotor)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {}

  override fun setRollerPower(outputPower: Double) {}
}
