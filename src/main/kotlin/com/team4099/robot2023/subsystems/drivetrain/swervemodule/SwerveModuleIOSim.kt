package com.team4099.robot2023.subsystems.drivetrain.swervemodule

import com.team4099.lib.units.AngularAcceleration
import com.team4099.lib.units.AngularVelocity
import com.team4099.lib.units.LinearAcceleration
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.celsius
import com.team4099.lib.units.base.inAmperes
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.inRotations
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.littletonrobotics.junction.Logger

class SwerveModuleIOSim(override val label: String) : SwerveModuleIO {
  // Use inverses of gear ratios because our standard is <1 is reduction
  private val driveMotorSim: FlywheelSim =
    FlywheelSim(DCMotor.getNEO(1), 1 / DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO, 0.025)

  private val steerMotorSim =
    FlywheelSim(
      DCMotor.getNEO(1), 1 / DrivetrainConstants.STEERING_SENSOR_GEAR_RATIO, 0.004096955
    )

  var turnRelativePosition = 0.0.radians
  var turnAbsolutePosition =
    (Math.random() * 2.0 * Math.PI).radians // getting a random value that we zero to
  var driveVelocity = 0.0.meters.perSecond

  private val driveFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_DRIVE_KP,
      DrivetrainConstants.PID.SIM_DRIVE_KI,
      DrivetrainConstants.PID.SIM_DRIVE_KD,
      Constants.Universal.LOOP_PERIOD_TIME.inSeconds
    )
  private val driveFeedForward =
    SimpleMotorFeedforward(
      DrivetrainConstants.PID.SIM_DRIVE_KS.inVolts, DrivetrainConstants.PID.SIM_DRIVE_KV.value
    )

  private val steeringFeedback =
    PIDController(
      DrivetrainConstants.PID.SIM_STEERING_KP,
      DrivetrainConstants.PID.SIM_STEERING_KI,
      DrivetrainConstants.PID.SIM_STEERING_KD,
      Constants.Universal.LOOP_PERIOD_TIME.inSeconds
    )

  init {
    steeringFeedback.enableContinuousInput(-Math.PI, Math.PI)
    steeringFeedback.setTolerance(DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR.inRadians)
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
        .inRotations
    inputs.steeringPosition = turnAbsolutePosition

    // s = r * theta -> d/2 * rad/s = m/s
    driveVelocity =
      (DrivetrainConstants.WHEEL_DIAMETER / 2 * driveMotorSim.angularVelocityRadPerSec).perSecond
    inputs.driveVelocity = driveVelocity

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

    // updating pid every loop cycle bc for some reason it doesn't stay like this otherwise
    driveFeedback.p = 0.9
    steeringFeedback.p = 23.0
  }

  // helper functions to clamp all inputs and set sim motor voltages properly
  private fun setDriveVoltage(volts: Double) {
    val driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
    driveMotorSim.setInputVoltage(driveAppliedVolts)
  }

  private fun setSteeringVoltage(volts: Double) {
    val turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
    steerMotorSim.setInputVoltage(turnAppliedVolts)
  }

  override fun setSteeringSetpoint(angle: Angle) {
    val feedback = steeringFeedback.calculate(turnAbsolutePosition.inRadians, angle.inRadians).volts
    Logger.getInstance().recordOutput("Drivetrain/PID/steeringFeedback", feedback.inVolts)
    Logger.getInstance().recordOutput("Drivetrain/PID/kP", steeringFeedback.p)
    Logger.getInstance().recordOutput("Drivetrain/PID/kI", steeringFeedback.i)
    Logger.getInstance().recordOutput("Drivetrain/PID/kD", steeringFeedback.d)
    setSteeringVoltage(feedback.inVolts)
  }

  override fun setClosedLoop(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration
  ) {
    val feedforward =
      driveFeedForward.calculate(speed.inMetersPerSecond, acceleration.inMetersPerSecondPerSecond)
        .volts

    setDriveVoltage(
      feedforward.inVolts +
        driveFeedback.calculate(driveVelocity.inMetersPerSecond, speed.inMetersPerSecond)
    )

    setSteeringSetpoint(steering)
  }

  override fun setOpenLoop(steering: Angle, power: Double) {
    setDriveVoltage(RoboRioSim.getVInVoltage() * power)
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

  override fun configureDrivePID(kP: Double, kI: Double, kD: Double) {
    driveFeedback.setPID(kP, kI, kD)
  }

  override fun configureSteeringPID(kP: Double, kI: Double, kD: Double) {
    steeringFeedback.setPID(kP, kI, kD)
  }

  override fun setBrakeMode(brake: Boolean) {
    println("Can't set brake mode in simulation")
  }

  override fun configureSteeringMotionMagic(
    maxVel: AngularVelocity,
    maxAccel: AngularAcceleration
  ) {
    println("Can't configure motion magic in simulation")
  }
}
