package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import org.littletonrobotics.junction.Logger
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
import org.team4099.lib.units.derived.volts
import java.lang.Math.PI

class SwerveModuleIOFalcon(
  private val steeringFalcon: TalonFX,
  private val driveFalcon: TalonFX,
  private val potentiometer: AnalogInput,
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

  private val potentiometerOutput: Double
    get() {
      return if (label != Constants.Drivetrain.BACK_RIGHT_MODULE_NAME) {
        potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
      } else {
        2 * PI - potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
      }
    }

  init {
    driveFalcon.configurator.apply(TalonFXConfiguration())
    steeringFalcon.configurator.apply(TalonFXConfiguration())

    driveFalcon.clearStickyFaults()
    steeringFalcon.clearStickyFaults()

    steeringConfiguration.Slot0.kP =
      steeringSensor.proportionalPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KP)
    steeringConfiguration.Slot0.kI =
      steeringSensor.integralPositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KI)
    steeringConfiguration.Slot0.kD =
      steeringSensor.derivativePositionGainToRawUnits(DrivetrainConstants.PID.STEERING_KD)
    steeringConfiguration.Slot0.kV
    =
      steeringSensor.velocityFeedforwardToRawUnits(DrivetrainConstants.PID.STEERING_KFF)
    steeringConfiguration.MotionMagic.MotionMagicCruiseVelocity =
      steeringSensor.velocityToRawUnits(DrivetrainConstants.STEERING_VEL_MAX)
    steeringConfiguration.MotionMagic.MotionMagicAcceleration =
      steeringSensor.accelerationToRawUnits(DrivetrainConstants.STEERING_ACCEL_MAX)
    steeringConfiguration.MotorOutput.PeakForwardDutyCycle = 1.0
    steeringConfiguration.MotorOutput.PeakForwardDutyCycle = -1.0
    steeringConfiguration.CurrentLimits.SupplyCurrentLimit =
      DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT.inAmperes
    steeringConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

    steeringConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake // change back to coast maybe?
    steeringFalcon.inverted = true
    steeringFalcon.configurator.apply(steeringConfiguration)

    driveConfiguration.Slot0.kP =
      driveSensor.proportionalVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KP)
    driveConfiguration.Slot0.kI =
      driveSensor.integralVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KI)
    driveConfiguration.Slot0.kD =
      driveSensor.derivativeVelocityGainToRawUnits(DrivetrainConstants.PID.DRIVE_KD)
    driveConfiguration.Slot0.kV = 0.05425
    //      driveSensor.velocityFeedforwardToRawUnits(DrivetrainConstants.PID.DRIVE_KFF)
    driveConfiguration.CurrentLimits.SupplyCurrentLimit  =
      DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.SupplyCurrentThreshold =
      DrivetrainConstants.DRIVE_THRESHOLD_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.SupplyTimeThreshold =
      DrivetrainConstants.DRIVE_TRIGGER_THRESHOLD_TIME.inSeconds
    driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    driveConfiguration.CurrentLimits.StatorCurrentLimit =
      DrivetrainConstants.DRIVE_STATOR_CURRENT_LIMIT.inAmperes
    driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = false // TODO tune

    driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    driveFalcon.configurator.apply(driveConfiguration)

    MotorChecker.add(
      "Drivetrain",
      "Drive",
      MotorCollection(

        mutableListOf(Falcon500(driveFalcon, "$label Drive Motor")),
        DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT - 30.amps,
        110.celsius
      )
    )

    MotorChecker.add(
      "Drivetrain",
      "Steering",
      MotorCollection(
        mutableListOf(Falcon500(steeringFalcon, "$label Steering Motor")),
        DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT,
        90.celsius,
        DrivetrainConstants.STEERING_SUPPLY_CURRENT_LIMIT - 10.amps,
        110.celsius
      )
    )
  }

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    inputs.driveAppliedVoltage = driveFalcon.motorOutputVoltage.volts
    inputs.steeringAppliedVoltage = steeringFalcon.motorOutputVoltage.volts

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

    inputs.potentiometerOutputRaw =
      potentiometer.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
    inputs.potentiometerOutputRadians = potentiometerOutput.radians

    Logger.getInstance()
      .recordOutput(
        "$label/potentiometerRadiansWithOffset",
        (inputs.potentiometerOutputRadians - zeroOffset).inRadians
      )

    Logger.getInstance()
      .recordOutput("$label/sensorVelocityRawUnits", driveFalcon.selectedSensorVelocity)
    Logger.getInstance().recordOutput("$label/motorOutput", driveFalcon.motorOutputPercent)
  }

  override fun setSteeringSetpoint(angle: Angle) {
    steeringFalcon.set(ControlMode.Position, steeringSensor.positionToRawUnits(angle))
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward = DrivetrainConstants.PID.DRIVE_KS * speed.sign
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
    println("Absolute Potentiometer Value $label ($potentiometerOutput")
  }

  override fun zeroSteering() {
    steeringFalcon.selectedSensorPosition =
      steeringSensor.positionToRawUnits(
        if (label != Constants.Drivetrain.BACK_RIGHT_MODULE_NAME)
          (potentiometerOutput.radians) - zeroOffset
        else (2 * PI).radians - (potentiometerOutput.radians - zeroOffset)
      )
    Logger.getInstance()
      .recordOutput("$label/zeroPositionRadians", steeringSensor.position.inRadians)
    println("Loading Zero for Module $label (${steeringFalcon.selectedSensorPosition})")
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

  override fun setDriveBrakeMode(brake: Boolean) {
    if (brake) {
      driveFalcon.setNeutralMode(NeutralMode.Brake)
    } else {
      driveFalcon.setNeutralMode(NeutralMode.Coast)
    }
  }

  override fun setSteeringBrakeMode(brake: Boolean) {
    if (brake) {
      steeringFalcon.setNeutralMode(NeutralMode.Brake)
    } else {
      steeringFalcon.setNeutralMode(NeutralMode.Coast)
    }
  }
}
