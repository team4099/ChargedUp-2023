package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.pathfollow.Trajectory
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.cos
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.inDegrees
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.derived.sin
import com.team4099.lib.units.inDegreesPerSecond
import com.team4099.lib.units.inDegreesPerSecondPerSecond
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.inRadiansPerSecond
import com.team4099.lib.units.inRadiansPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import kotlin.math.PI

class DrivePathCommand(
  val drivetrain: Drivetrain,
  private val trajectory: Trajectory,
  val resetPose: Boolean = false
) : CommandBase() {
  private val xPID: PIDController
  private val yPID: PIDController

  private val thetaPID: ProfiledPIDController

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  val thetakP = TunableNumber("Pathfollow/thetakP", DrivetrainConstants.PID.AUTO_THETA_PID_KP)
  val thetakI = TunableNumber("Pathfollow/thetakI", DrivetrainConstants.PID.AUTO_THETA_PID_KI)
  val thetakD = TunableNumber("Pathfollow/thetakD", DrivetrainConstants.PID.AUTO_THETA_PID_KD)

  val thetaMaxVel =
    TunableNumber(
      "Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL.inDegreesPerSecond
    )
  val thetaMaxAccel =
    TunableNumber(
      "Pathfollow/thetaMaxAccel",
      DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL.inDegreesPerSecondPerSecond
    )

  val poskP = TunableNumber("Pathfollow/poskP", DrivetrainConstants.PID.AUTO_POS_KP)
  val poskI = TunableNumber("Pathfollow/poskI", DrivetrainConstants.PID.AUTO_POS_KI)
  val poskD = TunableNumber("Pathfollow/poskD", DrivetrainConstants.PID.AUTO_POS_KD)

  init {
    addRequirements(drivetrain)

    xPID = PIDController(poskP.get(), poskD.get(), poskI.get())
    yPID = PIDController(poskP.get(), poskD.get(), poskI.get())
    thetaPID =
      ProfiledPIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
        TrapezoidProfile.Constraints(
          thetaMaxVel.get().degrees.perSecond.inRadiansPerSecond,
          thetaMaxAccel.get().degrees.perSecond.perSecond.inRadiansPerSecondPerSecond
        )
      )

    thetaPID.enableContinuousInput(-PI, PI)
  }

  override fun initialize() {
    if (resetPose) {
      drivetrain.odometryPose = trajectory.startingPose
    }
    trajStartTime = Clock.fpgaTime + trajectory.startTime
    thetaPID.reset(drivetrain.odometryPose.theta.inRadians)
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
        drivetrain.odometryPose.theta.inRadians,
        TrapezoidProfile.State(
          desiredState.pose.theta.inRadians,
          desiredState.angularVelocity.inRadiansPerSecond
        )
      )
        .radians
        .perSecond

    // Calculate feedback velocities (based on position error).
    val xFeedback =
      xPID.calculate(drivetrain.odometryPose.x.inMeters, desiredState.pose.x.inMeters)
        .meters
        .perSecond
    val yFeedback =
      yPID.calculate(drivetrain.odometryPose.y.inMeters, desiredState.pose.y.inMeters)
        .meters
        .perSecond

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
      .recordOutput("Pathfollow/thetaPIDPositionErrorRadians", thetaPID.positionError)
    Logger.getInstance()
      .recordOutput("Pathfollow/thetaPIDVelocityErrorRadians", thetaPID.velocityError)

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

    if (thetakP.hasChanged()) thetaPID.p = thetakP.get()
    if (thetakI.hasChanged()) thetaPID.i = thetakI.get()
    if (thetakD.hasChanged()) thetaPID.d = thetakD.get()

    if (poskP.hasChanged()) {
      xPID.p = poskP.get()
      yPID.p = poskP.get()
    }
    if (poskI.hasChanged()) {
      xPID.i = poskI.get()
      yPID.i = poskI.get()
    }
    if (poskD.hasChanged()) {
      xPID.d = poskD.get()
      yPID.d = poskD.get()
    }

    if (thetaMaxAccel.hasChanged() || thetaMaxVel.hasChanged()) {
      thetaPID.setConstraints(
        TrapezoidProfile.Constraints(
          thetaMaxVel.get().degrees.perSecond.inRadiansPerSecond,
          thetaMaxAccel.get().degrees.perSecond.perSecond.inRadiansPerSecondPerSecond
        )
      )
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
