package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.pathfollow.Trajectory
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.hal.Clock
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreesPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.step
import kotlin.math.PI

class DriveTrajectoryCommand(
  val drivetrain: Drivetrain,
  private val trajectorySupplier: () -> Trajectory,
  val resetPose: Boolean = false
) : CommandBase() {
  constructor(
    drivetrain: Drivetrain,
    trajectory: Trajectory,
    resetPose: Boolean = false
  ) : this(drivetrain, { trajectory }, resetPose) {}

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  private val thetaPID: ProfiledPIDController<Radian, Velocity<Radian>>

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  private lateinit var trajectory: Trajectory

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
    trajectory = trajectorySupplier()

    if (resetPose) {
      drivetrain.odometryPose = trajectory.startingPose
    }
    trajStartTime = Clock.fpgaTime + trajectory.startTime
    thetaPID.reset(drivetrain.odometryPose.rotation)
    xPID.reset()
    yPID.reset()

    val trajectoryArray = mutableListOf<Pose2d>()
    for (timeStep in 0.0.seconds..trajectory.duration step 0.25.seconds) {
      trajectoryArray.add(trajectory.sample(timeStep).pose.pose2d)
    }
    Logger.getInstance().recordOutput("Pathfollow/trajectory", *trajectoryArray.toTypedArray())
  }

  override fun execute() {
    trajCurTime = Clock.fpgaTime - trajStartTime
    val desiredState = trajectory.sample(trajCurTime)
    val xFF = desiredState.linearVelocity * desiredState.curvature.cos
    val yFF = desiredState.linearVelocity * desiredState.curvature.sin
    val thetaFeedback =
      thetaPID.calculate(
        drivetrain.odometryPose.rotation,
        TrapezoidProfile.State(desiredState.pose.rotation, desiredState.angularVelocity)
      )

    // Calculate feedback velocities (based on position error).
    val xFeedback = xPID.calculate(drivetrain.odometryPose.x, desiredState.pose.x)
    val yFeedback = yPID.calculate(drivetrain.odometryPose.y, desiredState.pose.y)

    val xAccel = desiredState.linearAcceleration * desiredState.curvature.cos
    val yAccel = desiredState.linearAcceleration * desiredState.curvature.sin

    drivetrain.targetPose = desiredState.pose

    drivetrain.setClosedLoop(
      thetaFeedback,
      Pair(xFF + xFeedback, yFF + yFeedback),
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
      .recordOutput("Pathfollow/Desired Angle in Degrees", desiredState.pose.rotation.inDegrees)
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
