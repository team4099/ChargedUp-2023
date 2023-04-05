package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TargetCorner
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.gameboy.objective.Objective
import com.team4099.robot2023.subsystems.gameboy.objective.isConeNode
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
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation2dWPILIB
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Translation3dWPILIB
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inRotations
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
  var nodeToLookFor: () -> Objective = { Objective() }

  val midConeNodePoses = mutableListOf<Pose3d>()
  val highConeNodePoses = mutableListOf<Pose3d>()

  // i think we need this for camera project to irl coordinates
  val vpw = (2.0 * (VisionConstants.Limelight.HORIZONTAL_FOV / 2).tan)
  val vph = (2.0 * (VisionConstants.Limelight.VERITCAL_FOV / 2).tan)

  private val xyStdDevCoefficient = TunableNumber("LimelightVision/xystdev", 0.05)
  private val thetaStdDev = TunableNumber("LimelightVision/thetaStdDev", 0.75)

  init{
    for (nodeIndex in listOf(0, 2, 3, 5, 6, 8)){
        midConeNodePoses.add(
          Pose3d(
            FieldConstants.Grids.midX,
            FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * nodeIndex,
            VisionConstants.Limelight.MID_TAPE_HEIGHT,
            Rotation3d()
          )
        )
        highConeNodePoses.add(
          Pose3d(
            FieldConstants.Grids.highX,
            FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * nodeIndex,
            VisionConstants.Limelight.HIGH_TAPE_HEIGHT,
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

    val visibleNodes = mutableListOf<Pose3d>()

    // mathematically figure out which nodes we're looking at based on limelight fov

    // process transform outputs from LL and get the target corners

    // calculating true node position
    val nodeX = when(nodeToLookFor.invoke().nodeTier){
      Constants.Universal.NodeTier.HYBRID -> FieldConstants.Grids.lowX
      Constants.Universal.NodeTier.MID -> FieldConstants.Grids.midX
      Constants.Universal.NodeTier.HIGH -> FieldConstants.Grids.highX
      else -> -1337.meters
    }

    val nodeColumn = if (FMSData.isBlue) nodeToLookFor.invoke().nodeColumn else 8 - nodeToLookFor.invoke().nodeColumn
    val nodeY = FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * nodeColumn

    val nodeZ = when(nodeToLookFor.invoke().nodeTier){
      Constants.Universal.NodeTier.HYBRID -> 0.0.meters
      Constants.Universal.NodeTier.MID -> VisionConstants.Limelight.MID_TAPE_HEIGHT
      Constants.Universal.NodeTier.HIGH -> VisionConstants.Limelight.HIGH_TAPE_HEIGHT
      else -> -1337.meters
    }

    val nodeRotation = Rotation3d(0.0.radians, 0.0.radians, if (FMSData.isBlue) 0.0.degrees else 180.degrees)

    val nodePose = Pose3d(nodeX, nodeY, nodeZ, nodeRotation)

    val robotPoses = mutableListOf<Pose2d>()


    if (inputs.validReading) {

      for (target in inputs.retroTargets) {
        visibleNodes.add(
          solveTargetPoseFromAngle(
            currentPose,
            target,
            nodeZ
          )
        )
      }
      // this is adding where we think they are,, not where they actually are
    }

    val timestampedVisionUpdates = mutableListOf<PoseEstimator.TimestampedVisionUpdate>()

    val trueVisibleNodes = mutableListOf<Pose3d>()

    for (node in visibleNodes) {
      val searchList = if(nodeZ == VisionConstants.Limelight.MID_TAPE_HEIGHT) midConeNodePoses else highConeNodePoses

      val closestPose = node.findClosestPose(*searchList.toTypedArray())

      trueVisibleNodes.add(closestPose)

      // find inverse translation from the detected pose to robot
      val targetToCamera = closestPose.relativeTo(
        currentPose.toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM)
      ).toTransform3d().inverse()

      val trueNodePoseToRobot = closestPose.transformBy(targetToCamera)

      // Add to vision updates
      val xyStdDev = xyStdDevCoefficient.get() * targetToCamera.translation.norm.inMeters.pow(2)
      val thetaStdDev = thetaStdDev.get() * targetToCamera.translation.norm.inMeters.pow(2)

      if (nodeToLookFor.invoke().isConeNode()){
        robotPoses.add(trueNodePoseToRobot.toPose2d())

        timestampedVisionUpdates.add(
          PoseEstimator.TimestampedVisionUpdate(
            inputs.timestamp,
            trueNodePoseToRobot.toPose2d(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
          )
        )
    }
    }

    Logger.getInstance().recordOutput("LimelightVision/estimatedRobotPoses", *robotPoses.map {it.pose2d} .toTypedArray())


    Logger.getInstance()
      .recordOutput(
        "LimelightVision/robotVisibleNodes", *visibleNodes.map { it.pose3d }.toTypedArray()
      )
    Logger.getInstance()
      .recordOutput(
        "LimelightVision/trueVisibleNodes", *trueVisibleNodes.map { it.pose3d }.toTypedArray()
      )

    Logger.getInstance().recordOutput("LimelightVision/cameraFieldRelativePose", currentPose.toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM).pose3d)

    Logger.getInstance().recordOutput("LoggedRobot/Subsystems/LimelightLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds)

//    visionConsumer.accept(timestampedVisionUpdates)
  }

  fun solveTargetPoseFromAngle(currentPose: Pose2d, target: LimelightReading, targetHeight: Length): Pose3d{
    val xyDistance = xyDistanceFromTarget(target, targetHeight)
    val distanceToTarget = hypot(xyDistance.inMeters, targetHeight.inMeters - VisionConstants.Limelight.LL_TRANSFORM.z.inMeters).meters

    val targetTranslation = Translation3dWPILIB(distanceToTarget.inMeters, Rotation3dWPILIB(0.0, -target.ty.inRadians, -target.tx.inRadians))

    Logger.getInstance().recordOutput("LimelightVision/distanceToTarget", distanceToTarget.inMeters)

    // figure out which way this target is facing using yaw of robot and yaw of camera
    val targetRotation =
      Rotation3d(0.0.degrees, 0.0.degrees,
        if (currentPose.rotation.rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z) in 0.degrees..180.degrees) {
          // we are looking at a red node which is facing towards 180 degrees
          180.0.degrees
        } else {
          // we are looking at a blue node which is facing towards 0 degrees
          0.degrees
        })

    return currentPose.toPose3d().transformBy(VisionConstants.Limelight.LL_TRANSFORM).transformBy(
      Transform3d(Translation3d(targetTranslation), targetRotation))
  }

  fun xyDistanceFromTarget(target: LimelightReading, targetHeight: Length): Length{
    var x = target.tx.tan
    var y = target.ty.tan
    var z = 1.0
    val normalVectorMagnitude = sqrt(x * x + y * y + z * z) // distance formula
    x /= normalVectorMagnitude
    y /= normalVectorMagnitude
    z /= normalVectorMagnitude

    val xPrime = x
    val yzPrime = Translation2d(y.meters, z.meters).rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.y)
    val yPrime = yzPrime.x
    val zPrime = yzPrime.y

    val angleToGoal = Math.asin(yPrime.inMeters)
    val targetToCameraHeight = targetHeight - VisionConstants.Limelight.LL_TRANSFORM.z

    return targetToCameraHeight / tan(angleToGoal)
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
      Rotation3d(0.0.degrees, 0.0.degrees,
      if (currentPose.rotation.rotateBy(cameraTransform.rotation.z) in 0.degrees..180.degrees) {
        // we are looking at a red node which is facing towards 180 degrees
        180.0.degrees
      } else {
        // we are looking at a blue node which is facing 0 degrees
        0.degrees
      })

    return Pose3d(currentPose
      .toPose3d()
      .transformBy(
        cameraTransform
      ).translation + translationFromTargetToCamera, targetRotation)
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
     currentPose.rotation.rotateBy(VisionConstants.Limelight.LL_TRANSFORM.rotation.z).rotateBy(180.degrees) // assuming we are looking at the tape head on

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
