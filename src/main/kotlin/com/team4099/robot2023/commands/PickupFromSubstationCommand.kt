package com.team4099.robot2023.commands

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.pathfollow.Path
import com.team4099.lib.pathfollow.Trajectory
import com.team4099.lib.pathfollow.Velocity2d
import com.team4099.lib.pathfollow.trajectoryFromPath
import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.commands.drivetrain.DriveTrajectoryCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.Substation
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees

class PickupFromSubstationCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val gamePiece: GamePiece,
  val substation: Substation
) : SequentialCommandGroup() {
  lateinit var trajectory: Trajectory
  var superstructureRequest: Request.SuperstructureRequest =
    when (substation) {
      Substation.DOUBLE_SUBSTATION_LEFT, Substation.DOUBLE_SUBSTATION_RIGHT ->
        Request.SuperstructureRequest.DoubleSubstationIntake(gamePiece)
      Substation.SINGLE_SUBSTATION ->
        Request.SuperstructureRequest.SingleSubstationIntake(gamePiece)
    }

  init {
    val setupCommand =
      runOnce(
        {
          val desiredPose =
            FieldConstants.allianceFlip(
              when (substation) {
                Substation.DOUBLE_SUBSTATION_LEFT -> {
                  Pose2d(
                    FieldConstants.idToTagPose(
                      Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID
                    )!!
                      .toPose2d()
                      .translation +
                      Translation2d(
                        -doubleSubstationXoffset.get(),
                        doubleSubstationYoffset.get()
                      ),
                    0.0.degrees
                  )
                }
                Substation.DOUBLE_SUBSTATION_RIGHT -> {
                  Pose2d(
                    FieldConstants.idToTagPose(
                      Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID
                    )!!
                      .toPose2d()
                      .translation +
                      Translation2d(
                        doubleSubstationXoffset.get(), doubleSubstationYoffset.get()
                      ),
                    0.0.degrees
                  )
                }
                Substation.SINGLE_SUBSTATION -> {
                  Pose2d(
                    FieldConstants.LoadingZone.singleSubstationTranslation +
                      Translation2d(
                        singleSubstationXoffset.get(),
                        -singleSubstationYoffset.get()
                      ),
                    90.0.degrees
                  )
                }
              }
            )

          val preDesiredPose =
            FieldConstants.allianceFlip(
              desiredPose +
                Transform2d(
                  when (substation) {
                    Substation.DOUBLE_SUBSTATION_LEFT,
                    Substation.DOUBLE_SUBSTATION_RIGHT -> {
                      Translation2d(
                        -preDesiredPoseDSXOffset.get(), preDesiredPoseDSYOffset.get()
                      )
                    }
                    Substation.SINGLE_SUBSTATION -> {
                      Translation2d(
                        preDesiredPoseSSXOffset.get(), -preDesiredPoseSSYOffset.get()
                      )
                    }
                  },
                  0.0.degrees
                )
            )

          trajectory =
            trajectoryFromPath(
              Path(
                drivetrain.odometryPose,
                desiredPose,
                drivetrain.fieldVelocity,
                Velocity2d()
              )
                .apply { addWaypoint(preDesiredPose.translation, desiredPose.rotation) },
              PathStore.trajectoryConfig
            ) // Heading is only equal to desiredPose's rotation
          // because
          //               we're facing the direction we're traveling
        },
        drivetrain,
        superstructure
      )

    addCommands(
      setupCommand,
      DriveTrajectoryCommand(drivetrain, { trajectory }),
      WaitCommand(1.5.seconds.inSeconds)
        .andThen(
          runOnce({ superstructure.currentRequest = superstructureRequest }, superstructure)
        )
    )
  }

  companion object {
    val doubleSubstationXoffset =
      LoggedTunableValue(
        "Drivetrain/doubleSubstationXOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
    val doubleSubstationYoffset =
      LoggedTunableValue(
        "Drivetrain/doubleSubstationYOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationXoffset =
      LoggedTunableValue(
        "Drivetrain/singleSubstationXOffset", 0.0.meters, Pair({ it.inInches }, { it.inches })
      )
    val singleSubstationYoffset =
      LoggedTunableValue(
        "Drivetrain/singleSubstationYOffset", 1.0.meters, Pair({ it.inInches }, { it.inches })
      )

    val preDesiredPoseDSXOffset =
      LoggedTunableValue(
        "Drivetrain/preDesiredPoseDSXOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
    val preDesiredPoseDSYOffset =
      LoggedTunableValue(
        "Drivetrain/preDesiredPoseDSYOffsetInches",
        0.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
    val preDesiredPoseSSXOffset =
      LoggedTunableValue(
        "Drivetrain/preDesiredPoseSSXOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
    val preDesiredPoseSSYOffset =
      LoggedTunableValue(
        "Drivetrain/preDesiredPoseSSYOffsetInches",
        1.0.meters,
        Pair({ it.inInches }, { it.inches })
      )
  }
}
