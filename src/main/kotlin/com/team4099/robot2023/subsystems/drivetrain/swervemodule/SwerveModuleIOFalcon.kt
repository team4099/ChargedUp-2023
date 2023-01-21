package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.AnalogPotentiometer
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians

class SwerveModuleIOFalcon(
  private val steeringFalcon: TalonFX,
  private val driveFalcon: TalonFX,
  private val potentiometer: AnalogPotentiometer,
  private val zeroOffset: Angle,
  override val label: String
) : SwerveModuleIO {
  private val steeringSensor =
    ctreAngularMechanismSensor(
      steeringFalcon,
      DrivetrainConstants.STEERING_SENSOR_CPR,
      DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO,
      DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE
    )
  private val driveSensor =
    ctreLinearMechanismSensor(
      driveFalcon,
      DrivetrainConstants.DRIVE_SENSOR_CPR,
      DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO,
      DrivetrainConstants.WHEEL_DIAMETER,
      DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE
    )

  // motor params
  private val steeringConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val driveConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val driveFeedforward =
    SimpleMotorFeedforward(
      DrivetrainConstants.PID.DRIVE_KS,
      DrivetrainConstants.PID.DRIVE_KV,
      DrivetrainConstants.PID.DRIVE_KA
    )

  init {
    driveFalcon.configFactoryDefault()
    steeringFalcon.configFactoryDefault()

    driveFalcon.clearStickyFaults()
    steeringFalcon.clearStickyFaults()

    steeringConfiguration.slot0.kP =
      steeringSensor.proportionalPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KP)
    steeringConfiguration.slot0.kI =
      steeringSensor.integralPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KI)
    steeringConfiguration.slot0.kD =
      steeringSensor.derivativePositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KD)
    steeringConfiguration.slot0.kF =
      steeringSensor.velocityFeedforwardToRawUnits(DrivetrainConstants.PID.STEERING_KFF)
    steeringConfiguration.motionCruiseVelocity =
      steeringSensor.velocityToRawUnits(DrivetrainConstants.STEERING_VEL_MAX)
    steeringConfiguration.motionAcceleration =
      steeringSensor.accelerationToRawUnits(DrivetrainConstants.STEERING_ACCEL_MAX)
    steeringConfiguration.peakOutputForward = 1.0
    steeringConfiguration.peakOutputReverse = -1.0
    steeringConfiguration.supplyCurrLimit.currentLimit =
      DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT.inAmperes
    steeringConfiguration.supplyCurrLimit.enable = true

    steeringFalcon.setNeutralMode(NeutralMode.Coast)
    steeringFalcon.inverted = true
    steeringFalcon.configAllSettings(steeringConfiguration)
    steeringFalcon.configAllowableClosedloopError(
      0, steeringSensor.positionToRawUnits(DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR)
    )

    driveConfiguration.slot0.kP =
      driveSensor.proportionalVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KP)
    driveConfiguration.slot0.kI =
      driveSensor.integralVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KI)
    driveConfiguration.slot0.kD =
      driveSensor.derivativeVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KD)
    driveConfiguration.slot0.kF = DrivetrainConstants.PID.DRIVE_KFF
    driveConfiguration.supplyCurrLimit.currentLimit =
      DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT.inAmperes
    driveConfiguration.supplyCurrLimit.triggerThresholdCurrent =
      DrivetrainConstants.DRIVE_THRESHOLD_CURRENT_LIMIT.inAmperes
    driveConfiguration.supplyCurrLimit.triggerThresholdTime =
      DrivetrainConstants.DRIVE_TRIGGER_THRESHOLD_TIME.inSeconds
    driveConfiguration.supplyCurrLimit.enable = true
    driveConfiguration.statorCurrLimit.currentLimit =
      DrivetrainConstants.DRIVE_STATOR_CURRENT_LIMIT.inAmperes
    driveConfiguration.statorCurrLimit.triggerThresholdCurrent =
      DrivetrainConstants.DRIVE_STATOR_THRESHOLD_CURRENT_LIMIT.inAmperes
    driveConfiguration.statorCurrLimit.triggerThresholdTime =
      DrivetrainConstants.DRIVE_STATOR_TRIGGER_THRESHOLD_TIME.inSeconds
    driveConfiguration.statorCurrLimit.enable = false // TODO tune
    driveConfiguration.voltageCompSaturation = 12.0

    driveFalcon.configAllSettings(driveConfiguration)
    driveFalcon.enableVoltageCompensation(true)

    driveFalcon.setNeutralMode(NeutralMode.Brake)
  }

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    //    inputs.driveAppliedVoltage = driveFalcon.motorOutputVoltage.volts
    //    inputs.steeringAppliedVoltage = steeringFalcon.motorOutputVoltage.volts

    inputs.driveStatorCurrent = driveFalcon.statorCurrent.amps
    inputs.driveSupplyCurrent = driveFalcon.supplyCurrent.amps
    inputs.steeringStatorCurrent = steeringFalcon.statorCurrent.amps
    inputs.steeringSupplyCurrent = steeringFalcon.statorCurrent.amps

    inputs.drivePosition = driveSensor.position
    inputs.steeringPosition = steeringSensor.position

    inputs.driveVelocity = driveSensor.velocity
    inputs.steeringVelocity = steeringSensor.velocity

    inputs.driveTemp = driveFalcon.temperature.celsius
    inputs.steeringTemp = steeringFalcon.temperature.celsius

    inputs.potentiometerOutputRaw = potentiometer.get()
    inputs.potentiometerOutputRadians = potentiometer.get().radians
  }

  override fun setSteeringSetpoint(angle: Angle) {
    steeringFalcon.set(ControlMode.Position, steeringSensor.positionToRawUnits(angle))
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward = driveFeedforward.calculate(speed, acceleration)
    driveFalcon.set(
      ControlMode.Velocity,
      driveSensor.velocityToRawUnits(speed),
      DemandType.ArbitraryFeedForward,
      feedforward.inVolts / 12.0
    )
    setSteeringSetpoint(steering)
  }

  /**
   * Open Loop Control using PercentOutput control on a Falcon
   *
   * @param steering: Desired angle
   * @param speed: Desired speed
   */
  override fun setOpenLoop(steering: Angle, speed: LinearVelocity) {
    driveFalcon.set(ControlMode.PercentOutput, speed / DrivetrainConstants.DRIVE_SETPOINT_MAX)
    setSteeringSetpoint(steering)
  }

  override fun resetModuleZero() {
    println("Absolute Potentiometer Value $label (${potentiometer.get()}")
  }

  override fun zeroSteering() {
    steeringFalcon.selectedSensorPosition =
      steeringSensor.positionToRawUnits(
        -(potentiometer.get().radians) + zeroOffset.inRadians.radians
      )
    println(
      "Loading Zero for Module $label (${steeringSensor.positionToRawUnits(
        -(potentiometer.get().radians) + zeroOffset.inRadians.radians
      )})"
    )
  }

  override fun zeroDrive() {
    driveFalcon.selectedSensorPosition = 0.0
  }

  override fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>
  ) {
    driveFalcon.config_kP(0, driveSensor.proportionalVelocityGainToRawUnits(kP))
    driveFalcon.config_kI(0, driveSensor.integralVelocityGainToRawUnits(kI))
    driveFalcon.config_kD(0, driveSensor.derivativeVelocityGainToRawUnits(kD))
  }

  override fun configureSteeringPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    steeringFalcon.config_kP(0, steeringSensor.proportionalPositionGainToRawUnits(kP))
    steeringFalcon.config_kI(0, steeringSensor.integralPositionGainToRawUnits(kI))
    steeringFalcon.config_kD(0, steeringSensor.derivativePositionGainToRawUnits(kD))
  }

  override fun configureSteeringMotionMagic(
    maxVel: AngularVelocity,
    maxAccel: AngularAcceleration
  ) {
    steeringConfiguration.motionCruiseVelocity = steeringSensor.velocityToRawUnits(maxVel)
    steeringConfiguration.motionAcceleration = steeringSensor.accelerationToRawUnits(maxAccel)
  }

  override fun setBrakeMode(brake: Boolean) {
    if (brake) {
      driveFalcon.setNeutralMode(NeutralMode.Brake)
    } else {
      driveFalcon.setNeutralMode(NeutralMode.Coast)
    }
  }
}
