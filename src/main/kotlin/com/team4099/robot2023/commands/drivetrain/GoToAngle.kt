package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreesPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class GoToAngle(val drivetrain: Drivetrain) : CommandBase() {
  private val thetaPID: ProfiledPIDController<Radian, Velocity<Radian>>

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreesPerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val thetaMaxVel =
    LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
  val thetaMaxAccel =
    LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)

  init {
    addRequirements(drivetrain)

    if (RobotBase.isReal()) {
      thetakP.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KD)
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    }

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
    thetaPID.reset(drivetrain.odometryPose.rotation)
  }

  override fun execute() {
    val desiredAngle =
      if (drivetrain.odometryPose.rotation.absoluteValue <= 90.degrees) 0.0.degrees
      else 180.degrees

    Logger.getInstance().recordOutput("ActiveCommands/AutoLevelCommand", true)

    val thetaFeedback = thetaPID.calculate(drivetrain.odometryPose.rotation, desiredAngle)

    drivetrain.setOpenLoop(
      thetaFeedback, Pair(0.0.meters.perSecond, 0.0.meters.perSecond), fieldOriented = true
    )

    Logger.getInstance()
      .recordOutput("AutoLevel/CurrentYawDegrees", drivetrain.odometryPose.rotation.inDegrees)
    Logger.getInstance().recordOutput("AutoLevel/DesiredYawDegrees", desiredAngle.inDegrees)
    Logger.getInstance()
      .recordOutput("AutoLevel/thetaFeedbackDPS", thetaFeedback.inDegreesPerSecond)
  }

  override fun isFinished(): Boolean {
    val desiredAngle =
      if (drivetrain.odometryPose.rotation.absoluteValue <= 90.degrees) 0.0.degrees
      else 180.degrees

    return (drivetrain.odometryPose.rotation - desiredAngle).absoluteValue <
      DrivetrainConstants.PID.AUTO_THETA_ALLOWED_ERROR
  }

  override fun end(interrupted: Boolean) {
    drivetrain.setOpenLoop(0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond))
  }
}
