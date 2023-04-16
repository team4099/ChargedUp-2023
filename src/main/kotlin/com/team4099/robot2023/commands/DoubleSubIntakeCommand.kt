package com.team4099.robot2023.commands

import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import java.lang.Math.PI

class DoubleSubIntakeCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : SequentialCommandGroup() {
  var driveRotation = { drivetrain.odometryPose.rotation }
  var targetAngle: Angle = 0.0.degrees

  val thetaProfiledPidController =
    ProfiledPIDController(
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KP,
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KI,
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KD,
      TrapezoidProfile.Constraints(
        DrivetrainConstants.TURN_SETPOINT_MAX, 40.degrees.perSecond.perSecond
      )
    )

  init {
    thetaProfiledPidController.enableContinuousInput(-PI.radians, PI.radians)

    val setupCommand =
      runOnce(
        { targetAngle = if (FMSData.isBlue) 0.0.degrees else 180.degrees },
        drivetrain,
        superstructure
      )

    addCommands(
      setupCommand,
      ParallelCommandGroup(
        TeleopDriveCommand(
          driver,
          driveX,
          driveY,
          {
            thetaProfiledPidController.calculate(driveRotation.invoke()) /
              DrivetrainConstants.TURN_SETPOINT_MAX
          },
          slowMode,
          drivetrain
        ),
        WaitUntilCommand { (driveRotation.invoke() - targetAngle).absoluteValue <= 5.degrees }
          .andThen(superstructure.doubleSubConeCommand()),
      )
    )
  }
}
