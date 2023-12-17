package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.util.PoseEstimator
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Quaternion
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.pow

class Vision(vararg cameras: CameraIO) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  companion object {
    val ambiguityThreshold = 0.7
    val targetLogTime = 0.05.seconds
    val cameraPoses = VisionConstants.CAMERA_TRANSFORMS

    val xyStdDevCoeffecient = 0.05
    val thetaStdDevCoefficient = 1.5
  }

  private val xyStdDevCoefficient = TunableNumber("Vision/xystdev", xyStdDevCoeffecient)

  private val thetaStdDev = TunableNumber("Vision/thetaStdDev", thetaStdDevCoefficient)

  private var poseSupplier = Supplier<Pose2d> { Pose2d() }
  private var visionConsumer: Consumer<List<PoseEstimator.TimestampedVisionUpdate>> = Consumer {}
  private val lastFrameTimes = mutableMapOf<Int, Time>()
  private val lastTagDetectionTimes = mutableMapOf<Int, Time>()

  init {
    for (i in io.indices) {
      lastFrameTimes[i] = 0.0.seconds
    }
  }

  fun setDataInterfaces(
    poseSupplier: Supplier<Pose2d>,
    visionConsumer: Consumer<List<PoseEstimator.TimestampedVisionUpdate>>
  ) {
    this.poseSupplier = poseSupplier
    this.visionConsumer = visionConsumer
  }

  override fun periodic() {
    //    val tuningPosition = Pose3d(Pose3d(
    //      (43.125).inches,
    //      (108.375).inches,
    //      (18.22).inches,
    //      Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
    //    ).translation  + (Translation3d(45.625.inches, 1.3125.inches, 0.0.inches)),
    // Rotation3d()).toPose2d()
    //
    //    Logger.recordOutput("Vision/tuningPosition", tuningPosition.pose2d)

    val startTime = Clock.realTimestamp

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
    }

    var currentPose: Pose2d = poseSupplier.get()
    val robotPoses = mutableListOf<Pose2d>()
    val visionUpdates = mutableListOf<PoseEstimator.TimestampedVisionUpdate>()

    for (instance in io.indices) {

      for (frameIndex in inputs[instance].timestamps.indices) {
        lastFrameTimes[instance] = Clock.fpgaTime
        val timestamp = inputs[instance].timestamps[frameIndex]
        val values = inputs[instance].frames[frameIndex]

        var cameraPose: Pose3d? = null
        var robotPose: Pose2d? = null

        when (values[0]) {
          1.0 -> {
            cameraPose =
              Pose3d(
                values[2].meters,
                values[3].meters,
                values[4].meters,
                Rotation3d(Quaternion(values[5].radians, values[6], values[7], values[8]))
              )

            //
            // Logger.recordOutput("Vision/${VisionConstants.CAMERA_NAMES[instance]}_transform",
            // cameraPose.relativeTo(tuningPosition.toPose3d()).pose3d)

            robotPose = cameraPose.transformBy(cameraPoses[instance].inverse()).toPose2d()
            println(
              "CameraPoseX: ${cameraPose.x}, transformX: ${cameraPoses[instance].x}, robotPoseX: ${robotPose.x}"
            )
          }
          2.0 -> {
            val error0 = values[1]
            val error1 = values[9]

            var use0 = false
            var use1 = false

            if (error0 < error1 * ambiguityThreshold) {
              use0 = true
            } else if (error1 < error0 * ambiguityThreshold) {
              use1 = true
            }

            if (use0) {
              cameraPose =
                Pose3d(
                  values[2].meters,
                  values[3].meters,
                  values[4].meters,
                  Rotation3d(Quaternion(values[5].radians, values[6], values[7], values[8]))
                )

              robotPose = cameraPose.transformBy(cameraPoses[instance].inverse()).toPose2d()
            } else if (use1) {
              cameraPose =
                Pose3d(
                  values[10].meters,
                  values[11].meters,
                  values[12].meters,
                  Rotation3d(
                    Quaternion(values[13].radians, values[14], values[15], values[16])
                  )
                )

              robotPose = cameraPose.transformBy(cameraPoses[instance].inverse()).toPose2d()
            } else {

              val cameraPose0 =
                Pose3d(
                  values[2].meters,
                  values[3].meters,
                  values[4].meters,
                  Rotation3d(Quaternion(values[5].radians, values[6], values[7], values[8]))
                )

              val robotPose0 = cameraPose0.transformBy(cameraPoses[instance].inverse()).toPose2d()

              val cameraPose1 =
                Pose3d(
                  values[10].meters,
                  values[11].meters,
                  values[12].meters,
                  Rotation3d(
                    Quaternion(values[13].radians, values[14], values[15], values[16])
                  )
                )

              val robotPose1 = cameraPose1.transformBy(cameraPoses[instance].inverse()).toPose2d()

              if (robotPose0.rotation.minus(currentPose.rotation).absoluteValue <
                robotPose1.rotation.minus(currentPose.rotation).absoluteValue
              ) {
                cameraPose = cameraPose0
                robotPose = robotPose0
              } else {
                cameraPose = cameraPose1
                robotPose = robotPose1
              }

              //
              // Logger.recordOutput("Vision/${VisionConstants.CAMERA_NAMES[instance]}_transform",
              // cameraPose.relativeTo(tuningPosition.toPose3d()).pose3d)
            }
          }
        }

        if (cameraPose == null || robotPose == null) {
          continue
        }

        if ((robotPose.rotation - currentPose.rotation).absoluteValue > 7.degrees &&
          DriverStation.isEnabled()
        ) {
          continue
        }

        // Find all detected tag poses
        val tagPoses = mutableListOf<Pose3d>()
        for (i in (if (values[0] == 1.0) 9 else 17) until values.size) {
          val tagId: Int = values[i].toInt()
          lastTagDetectionTimes[tagId] = Clock.fpgaTime
          // TODO: Convert detected tag to the actual pose for 2024
        }

        // Calculate average distance to tag
        var totalDistance = 0.0.meters
        for (tagPose in tagPoses) {
          totalDistance += tagPose.translation.getDistance(cameraPose.translation)
        }
        val averageDistance = totalDistance / tagPoses.size

        // Add to vision updates
        val xyStdDev = xyStdDevCoefficient.get() * averageDistance.inMeters.pow(2) / tagPoses.size
        val thetaStdDev = thetaStdDev.get() * averageDistance.inMeters.pow(2) / tagPoses.size

        visionUpdates.add(
          PoseEstimator.TimestampedVisionUpdate(
            timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
          )
        )
        robotPoses.add(robotPose)

        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/latencyMS",
          (Clock.fpgaTime - timestamp).inMilliseconds
        )

        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/estimatedRobotPose", robotPose.pose2d
        )

        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/tagPoses",
          *tagPoses.map { it.pose3d }.toTypedArray()
        )
      }

      if (inputs[instance].timestamps.isEmpty()) {
        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/estimatedRobotPose", Pose2d().pose2d
        )
      }

      if (Clock.fpgaTime - lastFrameTimes[instance]!! > targetLogTime) {
        Logger.recordOutput(
          "Vision/${VisionConstants.CAMERA_NAMES[instance]}/tagPoses", *arrayOf<Pose3dWPILIB>()
        )
      }
    }

    val allTagPoses = mutableListOf<Pose3d>()
    //    for (detectionEntry in lastTagDetectionTimes.entries) {
    //      if (Clock.fpgaTime - detectionEntry.value < targetLogTime) {
    //        FieldConstants.getTagPose(detectionEntry.key)?.let { allTagPoses.add(it) }
    //      }
    //    }

    Logger.recordOutput("Vision/allTagPoses", *allTagPoses.map { it.pose3d }.toTypedArray())

    visionConsumer.accept(visionUpdates)

    Logger.recordOutput(
      "LoggedRobot/Subsystems/VisionLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds
    )
  }
}
