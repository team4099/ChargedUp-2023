package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TargetCorner
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.gameboy.objective.Objective
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.PoseEstimator
import com.team4099.robot2023.util.findClosestPose
import com.team4099.robot2023.util.rotateBy
import com.team4099.robot2023.util.toPose3d
import com.team4099.robot2023.util.toTransform3d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Rotation3dWPILIB
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Translation3dWPILIB
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.tan
import java.util.function.Consumer
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

class LimelightVision(val io: LimelightVisionIO) : SubsystemBase() {
  val inputs = LimelightVisionIO.LimelightVisionIOInputs()

  var poseSupplier: () -> Pose2d = { Pose2d() }
  var visionConsumer: Consumer<List<PoseEstimator.TimestampedVisionUpdate>> = Consumer {}
  var gamePieceToLookFor: () -> Objective = { Objective() }

  val conePoses = mutableListOf<Pose3d>()
  val cubePoses = mutableListOf<Pose3d>()

  // i think we need this for camera project to irl coordinates
  val vpw = (2.0 * (VisionConstants.Limelight.HORIZONTAL_FOV / 2).tan)
  val vph = (2.0 * (VisionConstants.Limelight.VERITCAL_FOV / 2).tan)

  private val xyStdDevCoefficient = TunableNumber("LimelightVision/xystdev", 0.05)
  private val thetaStdDev = TunableNumber("LimelightVision/thetaStdDev", 0.75)

  val limelightState: LimelightStates = LimelightStates.AUTO_POSE_ESTIMATION

  var targetGamePiecePose = Pose3d()

  enum class LimelightStates {
    AUTO_POSE_ESTIMATION,
    TELEOP_GAME_PIECE_DETECTION
  }
  init {
    for (locationIndex in listOf(0, 1, 2, 3)) {
      conePoses.add(
        Pose3d(
          FieldConstants.StagingLocations.positionX,
          FieldConstants.StagingLocations.translations[locationIndex]?.y ?: 0.meters,
          VisionConstants.Limelight.CONE_HEIGHT,
          Rotation3d()
        )
      )

      cubePoses.add(
        Pose3d(
          FieldConstants.StagingLocations.positionX,
          FieldConstants.StagingLocations.translations[locationIndex]?.y ?: 0.meters,
          VisionConstants.Limelight.CUBE_HEIGHT,
          Rotation3d()
        )
      )
    }
  }

  override fun periodic() {
    val startTime = Clock.realTimestamp

    io.updateInputs(inputs)
    Logger.getInstance().processInputs("LimelightVision", inputs)

    var currentPose: Pose2d = poseSupplier.invoke()

    val visibleGamePieces = mutableListOf<Pose3d>()

    val timestampedVisionUpdates = mutableListOf<PoseEstimator.TimestampedVisionUpdate>()

    // mathematically figure out which game pieces we're looking at based on limelight neural
    // network detector

    // process transform outputs from LL and get the target corners

    if (limelightState == LimelightStates.AUTO_POSE_ESTIMATION) {
      // calculating true game piece position
      val gamePieceX = FieldConstants.StagingLocations.positionX

      val gamePieceLocationIndex =
        if (FMSData.isBlue) gamePieceToLookFor.invoke().autoStagingLocation
        else 3 - gamePieceToLookFor().autoStagingLocation

      val gamePieceY =
        FieldConstants.StagingLocations.firstY +
          FieldConstants.StagingLocations.separationY * gamePieceLocationIndex

      val gamePieceZ =
        when (gamePieceToLookFor.invoke().gamePiece) {
          Constants.Universal.GamePiece.CONE -> VisionConstants.Limelight.CONE_HEIGHT / 2
          Constants.Universal.GamePiece.CUBE -> VisionConstants.Limelight.CUBE_HEIGHT / 2
          else -> -1337.meters
        }

      val gamePieceRotation =
        Rotation3d(0.0.radians, 0.0.radians, if (FMSData.isBlue) 0.0.degrees else 180.degrees)

      val gamePiecePose = Pose3d(gamePieceX, gamePieceY, gamePieceZ, gamePieceRotation)

      val robotPoses = mutableListOf<Pose2d>()

      if (inputs.validReading) {

        for (target in inputs.gamePieceTargets) {
          visibleGamePieces.add(
            solveTargetPoseFromAngle(
              currentPose,
              target,
              (
                if (target.className == "cone") VisionConstants.Limelight.CONE_HEIGHT / 2
                else VisionConstants.Limelight.CUBE_HEIGHT / 2
                )
            )
          )
        }
        // this is adding where we think they are,, not where they actually are
      }

      val trueGamePieces = mutableListOf<Pose3d>()

      for ((index, gamePiecePose) in visibleGamePieces.withIndex()) {
        val searchList =
          if (inputs.gamePieceTargets[index].className == "cone") conePoses else cubePoses
        val closestPose = gamePiecePose.findClosestPose(*searchList.toTypedArray())

        trueGamePieces.add(closestPose)

        // find inverse translation from the detected pose to robot
        val targetToCamera =
          gamePiecePose
            .relativeTo(
              currentPose.toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM)
            )
            .toTransform3d()
            .inverse()

        val trueNodePoseToRobot = closestPose.transformBy(targetToCamera)

        // Add to vision updates
        val xyStdDev = xyStdDevCoefficient.get() * targetToCamera.translation.norm.inMeters.pow(2)
        val thetaStdDev = thetaStdDev.get() * targetToCamera.translation.norm.inMeters.pow(2)

        robotPoses.add(trueNodePoseToRobot.toPose2d())

        timestampedVisionUpdates.add(
          PoseEstimator.TimestampedVisionUpdate(
            inputs.timestamp,
            trueNodePoseToRobot.toPose2d(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
          )
        )
      }

      Logger.getInstance()
        .recordOutput(
          "LimelightVision/estimatedRobotPoses", *robotPoses.map { it.pose2d }.toTypedArray()
        )

      Logger.getInstance()
        .recordOutput(
          "LimelightVision/distanceToGamePieceX", robotPoses.getOrNull(0)?.minus(conePoses.getOrNull(0)?.toPose2d() ?: Pose2d())?.translation?.translation2d?.x ?: 0.0
        )

      Logger.getInstance()
        .recordOutput(
          "LimelightVision/distanceToGamePieceY", robotPoses.getOrNull(0)?.minus(conePoses.getOrNull(0)?.toPose2d() ?: Pose2d())?.translation?.translation2d?.y ?: 0.0
        )


      visionConsumer.accept(timestampedVisionUpdates)
    } else if (limelightState == LimelightStates.TELEOP_GAME_PIECE_DETECTION) {
      if (inputs.validReading) {
        for (target in inputs.gamePieceTargets) {
          visibleGamePieces.add(
            solveTargetPoseFromAngle(
              currentPose,
              target,
              (
                if (target.className == "cone") VisionConstants.Limelight.CONE_HEIGHT / 2
                else VisionConstants.Limelight.CUBE_HEIGHT / 2
                )
            )
          )
        }

        val searchList =
          visibleGamePieces.filterIndexed({ index, pose ->
            inputs.gamePieceTargets[index].className ==
              gamePieceToLookFor.invoke().gamePiece.toClassName()
          })
        targetGamePiecePose = currentPose.toPose3d().findClosestPose(*searchList.toTypedArray())
      }
    }

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/RawLimelightReadingsTx",
        inputs.gamePieceTargets.map { it.tx.inDegrees }.toDoubleArray()
      )

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/RawLimelightReadingsTy",
        inputs.gamePieceTargets.map { it.ty.inDegrees }.toDoubleArray()
      )

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/robotVisiblePieces",
        *visibleGamePieces.map { it.pose3d }.toTypedArray()
      )

    Logger.getInstance()
      .recordOutput(
        "LimelightVision/cameraFieldRelativePose",
        currentPose.toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM).pose3d
      )

    Logger.getInstance()
      .recordOutput(
        "LoggedRobot/Subsystems/LimelightLoopTimeMS",
        (Clock.realTimestamp - startTime).inMilliseconds
      )
    Logger.getInstance().recordOutput("LimelightVision/LimeLightState", limelightState.name)
    Logger.getInstance()
      .recordOutput(
        "LimelightVision/test",
        Pose3d(
          FieldConstants.StagingLocations.positionX,
          FieldConstants.StagingLocations.translations[0]?.y ?: 0.meters,
          VisionConstants.Limelight.CONE_HEIGHT / 2,
          Rotation3d()
        )
          .relativeTo(
            LimelightVisionIOSim.poseSupplier
              .invoke()
              .toPose3d()
              .transformBy(VisionConstants.Limelight.LL_TRANSFORM)
          )
          .pose3d
      )
  }

  fun solveTargetPoseFromAngle(
    currentPose: Pose2d,
    target: LimelightReading,
    targetHeight: Length
  ): Pose3d {
    val xyDistance = xyDistanceFromTarget(target, targetHeight)
    val distanceToTarget =
      hypot(
        xyDistance.inMeters,
        targetHeight.inMeters - VisionConstants.Limelight.LL_TRANSFORM.z.inMeters
      )
        .meters

    val targetTranslation =
      Translation3dWPILIB(
        distanceToTarget.inMeters,
        Rotation3dWPILIB(0.0, -target.ty.inRadians, -target.tx.inRadians)
      )

    Logger.getInstance().recordOutput("LimelightVision/distanceToTarget", distanceToTarget.inMeters)

    // figure out which way this target is facing using yaw of robot and yaw of camera
    val targetRotation =
      Rotation3d(
        0.0.degrees,
        0.0.degrees,
        if (currentPose.rotation.rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z) in
          0.degrees..180.degrees
        ) {
          // we are looking at a red node which is facing towards 180 degrees
          0.degrees
        } else {
          // we are looking at a blue node which is facing towards 0 degrees

          180.degrees
        }
      )

    return currentPose
      .toPose3d()
      .transformBy(VisionConstants.Limelight.LL_TRANSFORM)
      .transformBy(Transform3d(Translation3d(targetTranslation), Rotation3d(0.degrees, 0.degrees, 180.0.degrees)))
  }

  fun xyDistanceFromTarget(target: LimelightReading, targetHeight: Length): Length {
    var x = target.tx.tan
    var y = target.ty.tan
    var z = 1.0
    val normalVectorMagnitude = sqrt(x * x + y * y + z * z) // distance formula
    x /= normalVectorMagnitude
    y /= normalVectorMagnitude
    z /= normalVectorMagnitude

    val xPrime = x
    val yzPrime =
      Translation2d(y.meters, z.meters)
        .rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.y)
    val yPrime = yzPrime.x
    val zPrime = yzPrime.y

    val angleToGoal = Math.asin(yPrime.inMeters)
    val targetToCameraHeight = targetHeight - VisionConstants.Limelight.LL_TRANSFORM.z

    return targetToCameraHeight / tan(angleToGoal)
  }

  fun angleYawFromTarget(currentPose: Pose2d, targetPose: Pose3d): Angle {
    val robotToTarget = targetPose.toPose2d().relativeTo(currentPose)
    return Math.asin((robotToTarget.y.inMeters / robotToTarget.translation.magnitude)).radians
  }

  // based off of angles
  fun solveTargetPositionFromAngularOutput(
    tx: Angle,
    ty: Angle,
    currentPose: Pose2d,
    cameraTransform: Transform3d,
    targetHeight: Length
  ): Pose3d {
    val horizontalAngleFromCamera = -tx
    val verticalAngleFromCamera = -ty

    // rotation from robot to the target in frame
    val rotationFromTargetToCamera =
      Rotation3d(0.0.degrees, verticalAngleFromCamera, horizontalAngleFromCamera)

    val xDistanceFromTargetToCamera =
      (targetHeight - cameraTransform.z) / verticalAngleFromCamera.tan

    val yDistanceFromTargetToCamera = xDistanceFromTargetToCamera * horizontalAngleFromCamera.tan

    val translationFromTargetToCamera =
      Translation3d(
        xDistanceFromTargetToCamera,
        yDistanceFromTargetToCamera,
        targetHeight - cameraTransform.z
      )

    // figure out which way this target is facing using yaw of robot and yaw of camera
    val targetRotation =
      Rotation3d(
        0.0.degrees,
        0.0.degrees,
        if (currentPose.rotation.rotateBy(cameraTransform.rotation.z) in
          0.degrees..180.degrees
        ) {
          // we are looking at a red node which is facing towards 180 degrees
          180.0.degrees
        } else {
          // we are looking at a blue node which is facing 0 degrees
          0.degrees
        }
      )

    return Pose3d(
      currentPose.toPose3d().transformBy(cameraTransform).translation +
        translationFromTargetToCamera,
      targetRotation
    )
  }

  // based off of pixel coordinates
  private fun pixelCoordsToNormalizedPixelCoords(pixelCoords: CoordinatePair): CoordinatePair {
    // we're defining a coordinate pair to be where 0,0 is (horizontal fov / 2, vertical fov / 2)
    // and 1,1 is 1 pixel shifted both vertically and horizontally
    // note that pixel coordinates is returned where 0,0 is upper left, pos x is down, and pos y is
    // right but we want pos x to be right and pos y to be up
    return CoordinatePair(
      1 / (VisionConstants.Limelight.RES_WIDTH / 2) *
        (pixelCoords.x - VisionConstants.Limelight.RES_WIDTH - 0.5),
      1 / (VisionConstants.Limelight.RES_HEIGHT / 2) *
        (pixelCoords.x - VisionConstants.Limelight.RES_HEIGHT - 0.5)
    )
  }

  // need to make sure that all the corners are from the same target before passing into this
  // function
  fun solveTargetPositionFromCameraOutput(
    currentPose: Pose2d,
    pixelPosition: CoordinatePair,
    cameraTransform: Transform3d,
    targetHeight: Length
  ): Pose3d {
    val normalizedCoordinates = pixelCoordsToNormalizedPixelCoords(pixelPosition)

    // i think we wanna do what is there above because we are getting absolute position of the node
    // relative to the origin
    val targetRotation =
      currentPose
        .rotation
        .rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z)
        .rotateBy(180.degrees) // assuming we are looking at the tape head on

    val cartesianPoseOfTarget =
      Pose3d(
        currentPose.x + (vpw / 2 * normalizedCoordinates.x).meters,
        currentPose.y + (vph / 2 * normalizedCoordinates.y).meters,
        targetHeight,
        Rotation3d(0.0.degrees, 0.0.degrees, targetRotation)
      )

    return cartesianPoseOfTarget
  }

  // assume top left, top right, bottom left, bottom right
  fun centerOfRectangle(corners: List<TargetCorner>): CoordinatePair? {
    var xPos: Double? = null
    var yPos: Double? = null
    if (corners.size == 4) {
      xPos = corners.map { it.x }.average()
      yPos = corners.map { it.y }.average()
    }

    return if (xPos != null && yPos != null) {
      CoordinatePair(xPos, yPos)
    } else {
      null
    }
  }

  fun setDataInterfaces(
    poseSupplier: () -> Pose2d,
    visionConsumer: Consumer<List<PoseEstimator.TimestampedVisionUpdate>>
  ) {
    this.poseSupplier = poseSupplier
    this.visionConsumer = visionConsumer
  }
}
