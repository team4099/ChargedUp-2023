package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.random.Random

class SwerveModuleIOSim(override val label: String) : SwerveModuleIO {
  // Use inverses of gear ratios because our standard is <1 is reduction
  private val driveMotorSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      1 / DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO,
      DrivetrainConstants.DRIVE_WHEEL_INERTIA.inKilogramsMeterSquared
    )

  private val steerMotorSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      1 / DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO,
      DrivetrainConstants.STEERING_WHEEL_INERTIA.inKilogramsMeterSquared
    )

  var turnRelativePosition = 0.0.radians
  var turnAbsolutePosition =
    (Math.random() * 2.0 * Math.PI).radians // getting a random value that we zero to
  var driveVelocity = 0.0.meters.perSecond
  var drift: Length = 0.0.meters

  private val driveFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_DRIVE_KP,
      DrivetrainConstants.PID.SIM_DRIVE_KI,
      DrivetrainConstants.PID.SIM_DRIVE_KD,
      Constants.Universal.LOOP_PERIOD_TIME
    )
  private val driveFeedForward =
    SimpleMotorFeedforward(
      DrivetrainConstants.PID.SIM_DRIVE_KS, DrivetrainConstants.PID.SIM_DRIVE_KV
    )

  private val steeringFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_STEERING_KP,
      DrivetrainConstants.PID.SIM_STEERING_KI,
      DrivetrainConstants.PID.SIM_STEERING_KD,
      Constants.Universal.LOOP_PERIOD_TIME
    )

  init {
    steeringFeedback.enableContinuousInput(-Math.PI.radians, Math.PI.radians)
    steeringFeedback.errorTolerance = DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR
  }

  override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
    driveMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    steerMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    val angleDifference: Angle =
      (steerMotorSim.angularVelocityRadPerSec * Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        .radians
    turnAbsolutePosition += angleDifference
    turnRelativePosition += angleDifference

    // constrains it to 2pi radians
    while (turnAbsolutePosition < 0.radians) {
      turnAbsolutePosition += (2.0 * Math.PI).radians
    }
    while (turnAbsolutePosition > (2.0 * Math.PI).radians) {
      turnAbsolutePosition -= (2.0 * Math.PI).radians
    }

    // s = r * theta -> d/2 * rad/s = m/s
    driveVelocity =
      (DrivetrainConstants.WHEEL_DIAMETER / 2 * driveMotorSim.angularVelocityRadPerSec).perSecond

    // simming drift
    var loopCycleDrift = 0.0.meters
    if (Constants.Tuning.SIMULATE_DRIFT && driveVelocity > 2.0.meters.perSecond) {
      loopCycleDrift =
        (Random.nextDouble() * Constants.Tuning.DRIFT_CONSTANT)
          .meters // 0.0005 is just a nice number that ended up working out for drift
    }
    drift += loopCycleDrift

    // pi * d * rotations = distance travelled
    inputs.drivePosition =
      inputs.drivePosition +
      DrivetrainConstants.WHEEL_DIAMETER *
      Math.PI *
      (
        driveMotorSim.angularVelocityRadPerSec *
          Constants.Universal.LOOP_PERIOD_TIME.inSeconds
        )
        .radians
        .inRotations +
      loopCycleDrift // adding a random amount of drift
    inputs.steeringPosition = turnAbsolutePosition
    inputs.drift = drift

    inputs.driveVelocity = driveVelocity
    inputs.steeringVelocity = steerMotorSim.angularVelocityRadPerSec.radians.perSecond

    inputs.driveAppliedVoltage = (-1337).volts
    inputs.driveSupplyCurrent = driveMotorSim.currentDrawAmps.amps
    inputs.driveStatorCurrent =
      (-1337).amps // no way to get applied voltage to motor so can't actually calculate stator
    // current

    inputs.driveTemp = (-1337).celsius
    inputs.steeringTemp = (-1337).celsius

    inputs.steeringAppliedVoltage = (-1337).volts
    inputs.steeringSupplyCurrent = steerMotorSim.currentDrawAmps.amps
    inputs.steeringStatorCurrent =
      (-1337).amps // no way to get applied voltage to motor so can't actually calculate stator
    // current

    inputs.potentiometerOutputRadians = turnAbsolutePosition
    inputs.potentiometerOutputRaw = turnAbsolutePosition.inRadians

    // Setting a more accurate simulated voltage under load
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        inputs.driveSupplyCurrent.inAmperes + inputs.steeringSupplyCurrent.inAmperes
      )
    )
  }

  // helper functions to clamp all inputs and set sim motor voltages properly
  private fun setDriveVoltage(volts: ElectricalPotential) {
    val driveAppliedVolts = clamp(volts, -12.0.volts, 12.0.volts)
    Logger.getInstance().recordOutput("Drivetrain/appliedVolts", driveAppliedVolts.inVolts)
    driveMotorSim.setInputVoltage(driveAppliedVolts.inVolts)
  }

  private fun setSteeringVoltage(volts: ElectricalPotential) {
    val turnAppliedVolts = clamp(volts, -12.0.volts, 12.0.volts)
    steerMotorSim.setInputVoltage(turnAppliedVolts.inVolts)
  }

  override fun setSteeringSetpoint(angle: Angle) {
    val feedback = steeringFeedback.calculate(turnAbsolutePosition, angle)
    Logger.getInstance().recordOutput("Drivetrain/PID/angleSetpoint", angle.inRadians)
    Logger.getInstance().recordOutput("Drivetrain/PID/steeringFeedback", feedback.inVolts)
    Logger.getInstance()
      .recordOutput("Drivetrain/PID/kP", steeringFeedback.proportionalGain.inVoltsPerDegree)
    Logger.getInstance()
      .recordOutput("Drivetrain/PID/kI", steeringFeedback.integralGain.inVoltsPerDegreeSeconds)
    Logger.getInstance()
      .recordOutput(
        "Drivetrain/PID/kD", steeringFeedback.derivativeGain.inVoltsPerDegreePerSecond
      )
    setSteeringVoltage(feedback)
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward = driveFeedForward.calculate(speed, acceleration)
    setDriveVoltage(feedforward + driveFeedback.calculate(driveVelocity, speed))

    setSteeringSetpoint(steering)
  }

  override fun setOpenLoop(steering: Angle, speed: LinearVelocity) {
    setDriveVoltage(
      RoboRioSim.getVInVoltage().volts * (speed / DrivetrainConstants.DRIVE_SETPOINT_MAX)
    )
    setSteeringSetpoint(steering)
  }

  override fun resetModuleZero() {
    println("Resetting your module's 0 doesn't do anything meaningful in sim :(")
  }

  override fun zeroDrive() {
    println("Zero drive do anything meaningful in sim")
  }

  override fun zeroSteering() {
    turnAbsolutePosition = 0.0.radians
  }

  override fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>
  ) {
    driveFeedback.setPID(kP, kI, kD)
  }

  override fun configureSteeringPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    steeringFeedback.setPID(kP, kI, kD)
  }

  override fun setDriveBrakeMode(brake: Boolean) {
    println("Can't set brake mode in simulation")
  }

  override fun configureSteeringMotionMagic(
    maxVel: AngularVelocity,
    maxAccel: AngularAcceleration
  ) {
    println("Can't configure motion magic in simulation")
  }
}
