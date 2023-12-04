package com.team4099.robot2023.commands

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreesPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

class AutoAlignAndIntakeCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val limelight: LimelightVision,
  val gamePiece: Constants.Universal.GamePiece
) : SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  var heading: Angle = 0.0.degrees

  private val alignPID: PIDController<Radian, Velocity<Radian>>

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  val alignkP =
    LoggedTunableValue(
      "AutoAlign/alignkP",
      DrivetrainConstants.PID.AUTO_ALIGN_KP,
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val alignkI =
    LoggedTunableValue(
      "AutoAlign/alignkI",
      DrivetrainConstants.PID.AUTO_ALIGN_KI,
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val alignkD =
    LoggedTunableValue(
      "AutoAlign/alignkD",
      DrivetrainConstants.PID.AUTO_ALIGN_KD,
      Pair(
        { it.inDegreesPerSecondPerDegreesPerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val poskPX =
    LoggedTunableValue(
      "Pathfollow/poskPX",
      DrivetrainConstants.PID.AUTO_POS_KPX,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskIX =
    LoggedTunableValue(
      "Pathfollow/poskIX",
      DrivetrainConstants.PID.AUTO_POS_KIX,
      Pair({ it.inMetersPerSecondPerMeterSecond }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskDX =
    LoggedTunableValue(
      "Pathfollow/poskDX",
      DrivetrainConstants.PID.AUTO_POS_KDX,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  val poskPY =
    LoggedTunableValue(
      "Pathfollow/poskPY",
      DrivetrainConstants.PID.AUTO_POS_KPY,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskIY =
    LoggedTunableValue(
      "Pathfollow/poskIY",
      DrivetrainConstants.PID.AUTO_POS_KIY,
      Pair({ it.inMetersPerSecondPerMeterSecond }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskDY =
    LoggedTunableValue(
      "Pathfollow/poskDY",
      DrivetrainConstants.PID.AUTO_POS_KDY,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  init {

    alignPID = PIDController(alignkP.get(), alignkI.get(), alignkD.get())
    alignPID.errorTolerance = DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR

    xPID = PIDController(poskPX.get(), poskIX.get(), poskDX.get())
    yPID = PIDController(poskPY.get(), poskIY.get(), poskDY.get())

    val setupCommand =
      runOnce({
        Logger.getInstance().recordOutput("Auto/isAutoDriving", true)
        superstructure.objective.gamePiece = gamePiece
        alignPID
      })

    val alignmentCommand =
      Commands.run({
        val angle =
          limelight.angleYawFromTarget(drivetrain.odometryPose, limelight.targetGamePiecePose)
        val rotationFeedback = alignPID.calculate(angle, 0.radians)

        drivetrain.setClosedLoop(
          ChassisSpeeds(0.0, 0.0, rotationFeedback.inRadiansPerSecond),
          ChassisAccels(
            0.0.meters.perSecond.perSecond,
            0.0.meters.perSecond.perSecond,
            0.0.radians.perSecond.perSecond
          )
            .chassisAccelsWPILIB
        )
        if (alignPID.isAtSetpoint) {
          this.end(false)
        }
      })

    val intakeCommand =
      Commands.run({
        val angle =
          limelight.angleYawFromTarget(drivetrain.odometryPose, limelight.targetGamePiecePose)

        val rotationFeedback = alignPID.calculate(angle, 0.radians)
        val xFeedback = xPID.calculate(drivetrain.odometryPose.x, limelight.targetGamePiecePose.x)
        val yFeedback = yPID.calculate(drivetrain.odometryPose.y, limelight.targetGamePiecePose.y)

        drivetrain.setClosedLoop(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback.inMetersPerSecond,
            yFeedback.inMetersPerSecond,
            rotationFeedback.inRadiansPerSecond,
            drivetrain.odometryPose.rotation.inRotation2ds
          ),
          ChassisAccels(
            0.0.meters.perSecond.perSecond,
            0.0.meters.perSecond.perSecond,
            0.0.radians.perSecond.perSecond
          )
            .chassisAccelsWPILIB
        )
      })

    addCommands(
      setupCommand, alignmentCommand, superstructure.groundIntakeConeCommand(), intakeCommand
    )
  }
}
