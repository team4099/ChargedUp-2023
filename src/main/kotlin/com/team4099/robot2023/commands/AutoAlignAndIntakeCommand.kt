package com.team4099.robot2023.commands

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.purelyTranslateBy
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
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
import kotlin.math.PI

class AutoAlignAndIntakeCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val limelight: LimelightVision,
  val gamePiece: Constants.Universal.GamePiece
) : SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  var heading: Angle = 0.0.degrees

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>

  private var yawLock = 0.degrees

  val intakeDistance = LoggedTunableValue("AutoAlign/intakeDistance",
    (-3).inches,
    Pair({ it.inInches }, { it.inches})
  )

  val thetakP =
    LoggedTunableValue(
      "AutoAlign/alignkP",
      DrivetrainConstants.PID.AUTO_THETA_PID_KP,
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "AutoAlign/alignkI",
      DrivetrainConstants.PID.AUTO_THETA_PID_KI,
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "AutoAlign/alignkD",
      DrivetrainConstants.PID.AUTO_THETA_PID_KD,
      Pair(
        { it.inDegreesPerSecondPerDegreesPerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val poskP =
    LoggedTunableValue(
      "AutoAlign/poskP",
      DrivetrainConstants.PID.AUTO_ALIGN_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "AutoAlign/poskI",
      DrivetrainConstants.PID.AUTO_ALIGN_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSecond }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "AutoAlign/poskD",
      DrivetrainConstants.PID.AUTO_ALIGN_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )


  init {

    thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    thetaPID.errorTolerance = DrivetrainConstants.ALLOWED_STEERING_ANGLE_ERROR

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())



    val setupCommand =
      runOnce({
        Logger.getInstance().recordOutput("Auto/isAutoDriving", true)
        superstructure.objective.gamePiece = gamePiece
        thetaPID
      })



    val intakeCommand =
     Commands.run({
        val intakeOffsetTranslation = Translation2d(DrivetrainConstants.DRIVETRAIN_WIDTH/2 + intakeDistance.get(), 0.0.meters)

        if (limelight.targetGamePiecePose != null || ((Clock.fpgaTime - limelight.lastSeen) <= 0.5.seconds)) {
          Logger.getInstance().recordOutput("AutoAlign/driveTrainOdometry", drivetrain.odometryPose.pose2d)
          Logger.getInstance().recordOutput("AutoAlign/TargetPose", limelight.targetGamePiecePose?.pose3d)
          Logger.getInstance().recordOutput("AutoAlign/Tx", limelight.targetGamePieceTx?.inDegrees ?: 0.0)

          val xFeedback = xPID.calculate(drivetrain.odometryPose.purelyTranslateBy(intakeOffsetTranslation).x, limelight.targetGamePiecePose?.x ?: 0.0.meters)
          val yFeedback = yPID.calculate(drivetrain.odometryPose.purelyTranslateBy(intakeOffsetTranslation).y, limelight.targetGamePiecePose?.y ?: 0.0.meters)
          val thetaFeedback = thetaPID.calculate(limelight.targetGamePieceTx ?: 0.0.degrees, 0.0.degrees)

          Logger.getInstance().recordOutput("AutoAlign/xError", xPID.error.inMeters)
          Logger.getInstance().recordOutput("AutoAlign/yError", yPID.error.inMeters)
          Logger.getInstance().recordOutput("AutoAlign/thetaError", thetaPID.error.inDegrees)

          if (thetaPID.error.absoluteValue >= 6.degrees){
            drivetrain.setClosedLoop(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                0.0,
                0.0,
                thetaFeedback.inRadiansPerSecond,
                drivetrain.odometryPose.rotation.inRotation2ds
              ),
              ChassisAccels(
                0.0.meters.perSecond.perSecond,
                0.0.meters.perSecond.perSecond,
                0.0.radians.perSecond.perSecond
              )
                .chassisAccelsWPILIB
            )
          } else {
            drivetrain.setClosedLoop(
              ChassisSpeeds(
                xFeedback.absoluteValue.inMetersPerSecond,
                0.0,
                thetaFeedback.inRadiansPerSecond,
              ),
              ChassisAccels(
                0.0.meters.perSecond.perSecond,
                0.0.meters.perSecond.perSecond,
                0.0.radians.perSecond.perSecond
              )
                .chassisAccelsWPILIB
            )
          }
        }
      })

    addCommands(
      setupCommand, superstructure.groundIntakeConeCommand(), intakeCommand
    )
  }
}
