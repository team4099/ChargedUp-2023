package com.team4099.robot2023.subsystems.groundintake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import com.team4099.robot2023.subsystems.falconspin.Neo
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
import org.team4099.lib.units.derived.asDrivenOverDriving
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
    CANSparkMax(Constants.Intake.ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val rollerSensor =
    sparkMaxAngularMechanismSensor(
      rollerSparkMax,
      GroundIntakeConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
      GroundIntakeConstants.VOLTAGE_COMPENSATION
    )

  private val armSensor =
    sparkMaxAngularMechanismSensor(
      armSparkMax,
      GroundIntakeConstants.ARM_OUTPUT_GEAR_RATIO.asDrivenOverDriving,
      GroundIntakeConstants.VOLTAGE_COMPENSATION
    )

  private val armEncoder = armSparkMax.encoder

  private val throughBoreEncoder = DutyCycleEncoder(Constants.Intake.REV_ENCODER_PORT)

  private val armPIDController: SparkMaxPIDController = armSparkMax.pidController

  // gets the reported angle from the through bore encoder
  private val encoderAbsolutePosition: Angle
    get() {
      var output =
        (
          (-throughBoreEncoder.absolutePosition.rotations) *
            GroundIntakeConstants.ARM_ENCODER_GEAR_RATIO.asDrivenOverDriving
          )

      if (output in (-55).degrees..0.0.degrees) {
        output -= 180.degrees
      }

      return output
    }

  // uses the absolute encoder position to calculate the arm position
  private val armAbsolutePosition: Angle
    get() {
      return (encoderAbsolutePosition - GroundIntakeConstants.ABSOLUTE_ENCODER_OFFSET).inDegrees
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

    rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast

    rollerSparkMax.burnFlash()

    armSparkMax.restoreFactoryDefaults()
    armSparkMax.clearFaults()

    armSparkMax.enableVoltageCompensation(GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts)
    armSparkMax.setSmartCurrentLimit(GroundIntakeConstants.ARM_CURRENT_LIMIT.inAmperes.toInt())
    armSparkMax.inverted = GroundIntakeConstants.ARM_MOTOR_INVERTED
    armSparkMax.idleMode = CANSparkMax.IdleMode.kBrake

    armSparkMax.burnFlash()

    MotorChecker.add(
      "Ground Intake",
      "Roller",
      MotorCollection(
        mutableListOf(Neo(rollerSparkMax, "Roller Motor")),
        GroundIntakeConstants.ROLLER_CURRENT_LIMIT,
        70.celsius,
        GroundIntakeConstants.ROLLER_CURRENT_LIMIT - 30.amps,
        90.celsius
      ),
    )

    MotorChecker.add(
      "Ground Intake",
      "Pivot",
      MotorCollection(
        mutableListOf(Neo(armSparkMax, "Pivot Motor")),
        GroundIntakeConstants.ARM_CURRENT_LIMIT,
        70.celsius,
        GroundIntakeConstants.ARM_CURRENT_LIMIT - 30.amps,
        90.celsius
      )
    )
  }

  override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
    inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps

    // BusVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
    // AppliedVoltage = percentOutput * BusVoltage
    // SupplyCurrent = (percentOutput * BusVoltage / BusVoltage) * StatorCurrent =
    // percentOutput * statorCurrent
    inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput
    inputs.rollerTemp = rollerSparkMax.motorTemperature.celsius

    inputs.armPosition = armSensor.position
    inputs.armAbsoluteEncoderPosition = armAbsolutePosition
    inputs.armVelocity = armSensor.velocity
    inputs.armAppliedVoltage = armSparkMax.busVoltage.volts * armSparkMax.appliedOutput
    inputs.armStatorCurrent = armSparkMax.outputCurrent.amps
    inputs.armTemp = armSparkMax.motorTemperature.celsius

    // same math as  rollersupplycurrent
    inputs.armSupplyCurrent = inputs.armStatorCurrent * armSparkMax.appliedOutput

    Logger.getInstance()
      .recordOutput(
        "GroundIntake/absoluteEncoderPositionDegrees", encoderAbsolutePosition.inDegrees
      )

    Logger.getInstance()
      .recordOutput(
        "GroundIntake/armSensorVelocity",
        armSparkMax.encoder.velocity *
          GroundIntakeConstants.ARM_OUTPUT_GEAR_RATIO.asDrivenOverDriving
      )
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollerSparkMax.setVoltage(
      clamp(
        voltage,
        -GroundIntakeConstants.VOLTAGE_COMPENSATION,
        GroundIntakeConstants.VOLTAGE_COMPENSATION
      )
        .inVolts
    )
  }

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSparkMax.setVoltage(voltage.inVolts)

    Logger.getInstance().recordOutput("GroundIntake/commandedVoltage", voltage.inVolts)
  }

  /**
   * Sets the arm to a desired angle, uses feedforward to account for external forces in the system
   * The armPIDController uses the previously set PID constants and ff to calculate how to get to
   * the desired position
   *
   * @param armPosition the desired angle to set the aerm to
   * @param feedforward the amount of volts to apply for feedforward
   */
  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    armPIDController.setReference(
      armSensor.positionToRawUnits(armPosition),
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.inVolts
    )
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
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armPIDController.p = armSensor.proportionalPositionGainToRawUnits(kP)
    armPIDController.i = armSensor.integralPositionGainToRawUnits(kI)
    armPIDController.d = armSensor.derivativePositionGainToRawUnits(kD)
  }

  /** recalculates the current position of the neo encoder using value from the absolute encoder */
  override fun zeroEncoder() {
    armEncoder.position = armSensor.positionToRawUnits(armAbsolutePosition)
  }

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }

  /**
   * Sets the arm motor brake mode
   *
   * @param brake if it brakes
   */
  override fun setArmBrakeMode(brake: Boolean) {
    if (brake) {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kBrake
    } else {
      rollerSparkMax.idleMode = CANSparkMax.IdleMode.kCoast
    }
  }
}
