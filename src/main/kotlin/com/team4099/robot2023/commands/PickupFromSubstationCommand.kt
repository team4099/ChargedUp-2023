package com.team4099.robot2023.commands

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.purelyTranslateBy
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.Substation
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class PickupFromSubstationCommand(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val gamePiece: GamePiece,
  val substation: Substation
) : SequentialCommandGroup() {
  lateinit var drivePose: Pose2d
  var superstructureRequest: Request.SuperstructureRequest =
    when (substation) {
      Substation.DOUBLE_SUBSTATION_LEFT, Substation.DOUBLE_SUBSTATION_RIGHT ->
        Request.SuperstructureRequest.DoubleSubstationIntakePrep(gamePiece)
      Substation.SINGLE_SUBSTATION ->
        Request.SuperstructureRequest.SingleSubstationIntake(gamePiece)
    }

  val desiredPose =
    FieldConstants.allianceFlip(
      when (substation) {
        Substation.DOUBLE_SUBSTATION_LEFT -> {
          Pose2d(
            FieldConstants.idToTagPose(Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID)!!
              .toPose2d()
              .translation +
              Translation2d(-doubleSubstationXoffset.get(), doubleSubstationYoffset.get()),
            0.0.degrees
          )
        }
        Substation.DOUBLE_SUBSTATION_RIGHT -> {
          Pose2d(
            FieldConstants.idToTagPose(Constants.AprilTagIds.BLUE_DOUBLE_SUBSTATION_ID)!!
              .toPose2d()
              .translation +
              Translation2d(doubleSubstationXoffset.get(), doubleSubstationYoffset.get()),
            0.0.degrees
          )
        }
        Substation.SINGLE_SUBSTATION -> {
          Pose2d(
            FieldConstants.LoadingZone.singleSubstationTranslation +
              Translation2d(singleSubstationXoffset.get(), -singleSubstationYoffset.get()),
            90.0.degrees
          )
        }
      }
    )

  val preDesiredPose =
    FieldConstants.allianceFlip(
      desiredPose.purelyTranslateBy(
        when (substation) {
          Substation.DOUBLE_SUBSTATION_LEFT, Substation.DOUBLE_SUBSTATION_RIGHT -> {
            Translation2d(-preDesiredPoseDSXOffset.get(), preDesiredPoseDSYOffset.get())
          }
          Substation.SINGLE_SUBSTATION -> {
            Translation2d(preDesiredPoseSSXOffset.get(), -preDesiredPoseSSYOffset.get())
          }
        }
      )
    )

  init {

    val setupCommand = runOnce({ drivePose = drivetrain.odometryPose }, drivetrain)

    addCommands(
      setupCommand,
      ParallelCommandGroup(
        DrivePathCommand(
          drivetrain,
          {
            listOf(
              Waypoint(
                drivePose.pose2d.translation,
                drivePose.relativeTo(preDesiredPose).rotation.inRotation2ds,
                drivePose.rotation.inRotation2ds
              ),
              Waypoint(
                preDesiredPose.translation.translation2d,
                desiredPose.rotation.inRotation2ds,
                preDesiredPose.rotation.inRotation2ds
              ),
              Waypoint(
                desiredPose.translation.translation2d,
                null,
                desiredPose.rotation.inRotation2ds
              )
            )
          },
          keepTrapping = true,
          flipForAlliances = false
        ),
        WaitCommand(5.0)
          .andThen(
            runOnce(
              { superstructure.currentRequest = superstructureRequest },
              superstructure
            )
          )
      ),
      WaitCommand(3.0)
        .andThen(
          runOnce(
            { superstructure.currentRequest = Request.SuperstructureRequest.Idle() },
            superstructure
          )
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
        0.0.meters,
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
