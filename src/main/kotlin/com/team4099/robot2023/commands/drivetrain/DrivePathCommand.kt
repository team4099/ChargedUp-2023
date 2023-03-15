package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.lib.trajectory.CustomTrajectoryGenerator
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.hal.Clock
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
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
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond
import java.util.function.Supplier
import kotlin.math.PI

class DrivePathCommand(
  val drivetrain: Drivetrain,
  private val waypoints: Supplier<List<Waypoint>>,
  val resetPose: Boolean = false,
  val keepTrapping: Boolean = false,
  val flipForAlliances: Boolean = true,
  val endVelocity: Velocity2d = Velocity2d(),
) : CommandBase() {
  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private val swerveDriveController: CustomHolonomicDriveController

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  var trajectoryGenerator = CustomTrajectoryGenerator()

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

  private fun generate(
    waypoints: List<Waypoint>,
    constraints: List<TrajectoryConstraint> = listOf()
  ) {
    // Set up trajectory configuration
    val config =
      edu.wpi.first.math.trajectory.TrajectoryConfig(
        DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond,
        DrivetrainConstants.MAX_AUTO_ACCEL.inMetersPerSecondPerSecond
      )
        .setKinematics(
          SwerveDriveKinematics(
            *(drivetrain.moduleTranslations.map { it.translation2d }).toTypedArray()
          )
        )
        .setStartVelocity(drivetrain.fieldVelocity.magnitude.inMetersPerSecond)
        .setEndVelocity(endVelocity.magnitude.inMetersPerSecond)
        .addConstraint(
          CentripetalAccelerationConstraint(
            DrivetrainConstants.STEERING_ACCEL_MAX.inRadiansPerSecondPerSecond
          )
        )
        .addConstraints(constraints)

    try {
      trajectoryGenerator.generate(config, waypoints)
    } catch (exception: TrajectoryGenerationException) {
      DriverStation.reportError("Failed to generate trajectory.", true)
    }
  }

  init {
    addRequirements(drivetrain)

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    swerveDriveController =
      CustomHolonomicDriveController(
        xPID.wpiPidController, yPID.wpiPidController, thetaPID.wpiPidController
      )

    swerveDriveController.setTolerance(Pose2d(0.5.inches, 0.5.inches, 2.5.degrees).pose2d)
  }

  override fun initialize() {
    // trajectory generation!
    generate(waypoints.get())

    val trajectory = trajectoryGenerator.driveTrajectory

    //    if (resetPose) {
    //      drivetrain.odometryPose = AllianceFlipUtil.apply(Pose2d(trajectory.initialPose))
    //    }
    trajStartTime = Clock.fpgaTime + trajectory.states[0].timeSeconds.seconds
    thetaPID.reset()
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    val trajectory = trajectoryGenerator.driveTrajectory

    if (trajectory.states.size <= 1) {
      return
    }

    trajCurTime = Clock.fpgaTime - trajStartTime
    var desiredState = trajectory.sample(trajCurTime.inSeconds)

    var desiredRotation =
      trajectoryGenerator.holonomicRotationSequence.sample(trajCurTime.inSeconds)

    if (flipForAlliances) {
      desiredState = AllianceFlipUtil.apply(desiredState)
      desiredRotation = AllianceFlipUtil.apply(desiredRotation)
    }

    val xAccel =
      desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
        desiredState.curvatureRadPerMeter.radians.cos
    val yAccel =
      desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
        desiredState.curvatureRadPerMeter.radians.sin

    val nextDriveState =
      swerveDriveController.calculate(
        drivetrain.odometryPose.pose2d, desiredState, desiredRotation
      )

    drivetrain.targetPose =
      Pose2d(Pose2dWPILIB(desiredState.poseMeters.translation, desiredRotation.position))

    Logger.getInstance()
      .recordOutput(
        "Pathfollow/target",
        Pose2dWPILIB(desiredState.poseMeters.translation, desiredRotation.position)
      )

    drivetrain.setClosedLoop(
      nextDriveState,
      ChassisAccels(xAccel, yAccel, 0.0.radians.perSecond.perSecond).chassisAccelsWPILIB
    )

    Logger.getInstance()
      .recordOutput("Pathfollow/thetaPIDPositionErrorRadians", thetaPID.error.inRadians)

    Logger.getInstance().recordOutput("Pathfollow/xPIDPositionErrorMeters", xPID.error.inMeters)
    Logger.getInstance().recordOutput("Pathfollow/yPIDPositionErrorMeters", yPID.error.inMeters)
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
      .recordOutput(
        "Pathfollow/Desired Angle in Degrees", desiredState.poseMeters.rotation.degrees
      )
    Logger.getInstance()
      .recordOutput(
        "Pathfollow/Desired Angular Velocity in Degrees",
        desiredRotation.velocityRadiansPerSec.radians.perSecond.inDegreesPerSecond
      )

    Logger.getInstance().recordOutput("Pathfollow/trajectory", trajectory)
    Logger.getInstance().recordOutput("Pathfollow/isAtReference", swerveDriveController.atReference())

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
  }

  override fun isFinished(): Boolean {
    trajCurTime = Clock.fpgaTime - trajStartTime
    return (!keepTrapping || swerveDriveController.atReference()) &&
      trajCurTime > trajectoryGenerator.driveTrajectory.totalTimeSeconds.seconds
  }

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      // Stop where we are if interrupted
      drivetrain.setOpenLoop(0.degrees.perSecond, Pair(0.meters.perSecond, 0.meters.perSecond))
    } else {
      // Execute one last time to end up in the final state of the trajectory
      // Since we weren't interrupted, we know curTime > endTime
      execute()
      drivetrain.setOpenLoop(0.degrees.perSecond, Pair(0.meters.perSecond, 0.meters.perSecond))
    }
  }
}
