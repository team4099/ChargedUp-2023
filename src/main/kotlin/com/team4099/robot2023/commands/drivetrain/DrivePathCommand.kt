package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.controller.PIDController
import com.team4099.lib.controller.ProfiledPIDController
import com.team4099.lib.controller.TrapezoidProfile
import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.pathfollow.Trajectory
import com.team4099.lib.units.Velocity
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.Radian
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import com.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import com.team4099.lib.units.derived.inDegreesPerSecondPerDegreesPerSecond
import com.team4099.lib.units.derived.inMetersPerSecondPerMeter
import com.team4099.lib.units.derived.inMetersPerSecondPerMeterSecond
import com.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import com.team4099.lib.units.derived.perDegree
import com.team4099.lib.units.derived.perDegreePerSecond
import com.team4099.lib.units.derived.perDegreeSeconds
import com.team4099.lib.units.derived.perMeter
import com.team4099.lib.units.derived.perMeterSeconds
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import com.team4099.lib.units.inDegreesPerSecond
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import kotlin.math.PI

class DrivePathCommand(
  val drivetrain: Drivetrain,
  private val trajectory: Trajectory,
  val resetPose: Boolean = false
) : CommandBase() {
  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  private val thetaPID: ProfiledPIDController<Radian, Velocity<Radian>>

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      DrivetrainConstants.PID.AUTO_THETA_PID_KP,
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      DrivetrainConstants.PID.AUTO_THETA_PID_KI,
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      DrivetrainConstants.PID.AUTO_THETA_PID_KD,
      Pair(
        { it.inDegreesPerSecondPerDegreesPerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val thetaMaxVel =
    LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
  val thetaMaxAccel =
    LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)

  val poskP =
    LoggedTunableValue(
      "Pathfollow/poskP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "Pathfollow/poskI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSecond }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "Pathfollow/poskD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  init {
    addRequirements(drivetrain)

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    thetaPID =
      ProfiledPIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
        TrapezoidProfile.Constraints(thetaMaxVel.get(), thetaMaxAccel.get())
      )

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    if (resetPose) {
      drivetrain.odometryPose = trajectory.startingPose
    }
    trajStartTime = Clock.fpgaTime + trajectory.startTime
    thetaPID.reset(drivetrain.odometryPose.theta)
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    trajCurTime = Clock.fpgaTime - trajStartTime
    val desiredState = trajectory.sample(trajCurTime)
    val xFF = desiredState.linearVelocity * desiredState.curvature.cos
    val yFF = desiredState.linearVelocity * desiredState.curvature.sin
    val thetaFeedback =
      thetaPID.calculate(
        drivetrain.odometryPose.theta,
        TrapezoidProfile.State(desiredState.pose.theta, desiredState.angularVelocity)
      )

    // Calculate feedback velocities (based on position error).
    val xFeedback = xPID.calculate(drivetrain.odometryPose.x, desiredState.pose.x)
    val yFeedback = yPID.calculate(drivetrain.odometryPose.y, desiredState.pose.y)

    val xAccel = desiredState.linearAcceleration * desiredState.curvature.cos
    val yAccel = desiredState.linearAcceleration * desiredState.curvature.sin

    drivetrain.targetPose = desiredState.pose

    drivetrain.setClosedLoop(
      -thetaFeedback,
      Pair(-xFF - xFeedback, yFF + yFeedback),
      0.radians.perSecond.perSecond,
      Pair(xAccel, yAccel),
      true,
    )

    Logger.getInstance().recordOutput("Pathfollow/xFFMetersPerSec", xFF.inMetersPerSecond)
    Logger.getInstance().recordOutput("Pathfollow/yFFMetersPerSec", yFF.inMetersPerSecond)
    Logger.getInstance()
      .recordOutput("Pathfollow/thetaFeedbackDegreesPerSec", thetaFeedback.inDegreesPerSecond)

    Logger.getInstance()
      .recordOutput("Pathfollow/xFeedbackMetersPerSec", xFeedback.inMetersPerSecond)
    Logger.getInstance()
      .recordOutput("Pathfollow/yFeedbackMetersPerSec", yFeedback.inMetersPerSecond)

    Logger.getInstance()
      .recordOutput("Pathfollow/thetaPIDPositionErrorRadians", thetaPID.error.inRadians)
    Logger.getInstance()
      .recordOutput(
        "Pathfollow/thetaPIDVelocityErrorRadians", thetaPID.errorDerivative.inRadiansPerSecond
      )

    Logger.getInstance()
      .recordOutput(
        "Pathfollow/xAccelMetersPerSecondPerSecond", xAccel.inMetersPerSecondPerSecond
      )
    Logger.getInstance()
      .recordOutput(
        "Pathfollow/yAccelMetersPerSecondPerSecond", yAccel.inMetersPerSecondPerSecond
      )

    Logger.getInstance().recordOutput("Pathfollow/Start Time", trajStartTime.inSeconds)
    Logger.getInstance().recordOutput("Pathfollow/Current Time", trajCurTime.inSeconds)
    Logger.getInstance()
      .recordOutput("Pathfollow/Desired Angle in Degrees", desiredState.pose.theta.inDegrees)
    Logger.getInstance()
      .recordOutput(
        "Pathfollow/Desired Angular Velocity in Degrees",
        desiredState.angularVelocity.inDegreesPerSecond
      )

    Logger.getInstance().recordOutput("ActiveCommands/DrivePathCommand", true)

    if (thetakP.hasChanged()) thetaPID.proportionalGain = thetakP.get()
    if (thetakI.hasChanged()) thetaPID.integralGain = thetakI.get()
    if (thetakD.hasChanged()) thetaPID.derivativeGain = thetakD.get()

    if (poskP.hasChanged()) {
      xPID.proportionalGain = poskP.get()
      yPID.proportionalGain = poskP.get()
    }
    if (poskI.hasChanged()) {
      xPID.integralGain = poskI.get()
      yPID.integralGain = poskI.get()
    }
    if (poskD.hasChanged()) {
      xPID.derivativeGain = poskD.get()
      yPID.derivativeGain = poskD.get()
    }

    if (thetaMaxAccel.hasChanged() || thetaMaxVel.hasChanged()) {
      thetaPID.setConstraints(TrapezoidProfile.Constraints(thetaMaxVel.get(), thetaMaxAccel.get()))
    }
  }

  override fun isFinished(): Boolean {
    trajCurTime = Clock.fpgaTime - trajStartTime
    return trajCurTime > trajectory.endTime
  }

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      // Stop where we are if interrupted
      drivetrain.setClosedLoop(0.degrees.perSecond, Pair(0.meters.perSecond, 0.meters.perSecond))
    } else {
      // Execute one last time to end up in the final state of the trajectory
      // Since we weren't interrupted, we know curTime > endTime
      execute()
      drivetrain.setClosedLoop(0.degrees.perSecond, Pair(0.meters.perSecond, 0.meters.perSecond))
    }
  }
}
