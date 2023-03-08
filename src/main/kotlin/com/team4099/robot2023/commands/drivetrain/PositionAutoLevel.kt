package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.isAboveMiddleOfChargeStation
import com.team4099.robot2023.util.isOnInnerSideOfChargeStation
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import kotlin.math.atan2

class PositionAutoLevel(val drivetrain: Drivetrain) : CommandBase() {

  lateinit var executeCommand: CommandBase

  override fun initialize() {

    val cornerIndex =
      if (drivetrain.odometryPose.isOnInnerSideOfChargeStation() &&
        !drivetrain.odometryPose.isAboveMiddleOfChargeStation()
      ) {
        0
      } else if (drivetrain.odometryPose.isOnInnerSideOfChargeStation() &&
        drivetrain.odometryPose.isAboveMiddleOfChargeStation()
      ) {
        1
      } else if (!drivetrain.odometryPose.isOnInnerSideOfChargeStation() &&
        !drivetrain.odometryPose.isAboveMiddleOfChargeStation()
      ) {
        2
      } else {
        3
      }

    val chargeStationTranslation =
      Translation2d(
        (
          (
            FieldConstants.Community.chargingStationCorners[3].x +
              FieldConstants.Community.chargingStationCorners[0].x
            ) / 2
          )
          .inMeters,
        (
          (
            FieldConstants.Community.chargingStationCorners[3].y +
              FieldConstants.Community.chargingStationCorners[0].y
            ) / 2
          )
          .inMeters
      )

    val intermediateTranslationXOffset =
      if (drivetrain.odometryPose.isOnInnerSideOfChargeStation()) -20.inches else 20.inches

    val intermediateTranslationYOffset =
      if (drivetrain.odometryPose.isAboveMiddleOfChargeStation()) -20.inches else 20.inches

    executeCommand =
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            Waypoint(
              drivetrain.odometryPose.translation.translation2d,
              if (drivetrain.odometryPose.isAboveMiddleOfChargeStation())
                -90.degrees.inRotation2ds
              else 90.degrees.inRotation2ds,
              drivetrain.odometryPose.rotation.inRotation2ds,
            ),
            Waypoint(
              AllianceFlipUtil.apply(
                Translation2d(
                  FieldConstants.Community.chargingStationCorners[cornerIndex]
                    .translation2d
                    .x + intermediateTranslationXOffset.inMeters,
                  FieldConstants.Community.chargingStationCorners[cornerIndex]
                    .translation2d
                    .y + intermediateTranslationYOffset.inMeters
                )
              ),
              atan2(
                chargeStationTranslation.y - drivetrain.odometryPose.y.inMeters,
                chargeStationTranslation.x - drivetrain.odometryPose.x.inMeters
              )
                .radians
                .inRotation2ds,
              null
            ),
            Waypoint(
              AllianceFlipUtil.apply(chargeStationTranslation),
              0.0.degrees.inRotation2ds,
              drivetrain.odometryPose.rotation.inRotation2ds
            )
          )
        },
        flipForAlliances = false,
        keepTrapping = true
      )

    executeCommand.initialize()
  }

  override fun execute() {
    executeCommand.execute()

    Logger.getInstance().recordOutput("ActiveCommands/PositionAutoLevelCommand", true)
  }

  override fun isFinished(): Boolean {
    if (executeCommand.isFinished) {
      Logger.getInstance().recordOutput("ActiveCommands/PositionAutoLevelCommand", false)
    }
    return executeCommand.isFinished
  }
}
