package com.team4099.robot2023.commands

import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond

class DoubleSubIntakeCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : SequentialCommandGroup() {
  var driveRotation: Angle = 0.0.degrees
  var targetAngle: Angle = 0.0.degrees

  val thetaProfiledPidController =
    ProfiledPIDController(
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KP,
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KI,
      DrivetrainConstants.PID.DOUBLE_SUB_AUTO_ALIGN_KD,
      TrapezoidProfile.Constraints(10.degrees.perSecond, 40.degrees.perSecond.perSecond)
    )

  init {
    val setupCommand =
      runOnce(
        {
          driveRotation = drivetrain.odometryPose.rotation
          targetAngle = if (FMSData.isBlue) 0.0.degrees else 180.degrees
        },
        drivetrain,
        superstructure
      )

    addCommands(
      setupCommand,
      RunCommand(
        {
          drivetrain.setOpenLoop(
            thetaProfiledPidController.calculate(
              drivetrain.odometryPose.rotation, targetAngle
            ),
            driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
          )
        },
        drivetrain
      )
    )
  }
}
