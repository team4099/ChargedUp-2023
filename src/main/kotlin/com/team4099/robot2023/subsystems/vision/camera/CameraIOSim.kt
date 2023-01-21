package com.team4099.robot2023.subsystems.vision.camera

import com.team4099.lib.sim.utils.estimation.CameraProperties
import com.team4099.lib.sim.utils.sim.PhotonCamera
import com.team4099.lib.sim.utils.sim.PhotonCameraSim
import com.team4099.lib.sim.utils.sim.SimVisionSystem
import com.team4099.lib.vision.VisionResult
import com.team4099.lib.vision.VisionTarget
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import org.littletonrobotics.junction.Logger
import org.team4099.lib.apriltag.AprilTagFieldLayout
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.milli

class CameraIOSim(override val transformToRobot: Transform3d, override val cameraName: String) :
  CameraIO {

  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  val ntInstance = NetworkTableInstance.getDefault()
  val dtPoseArraySub =
    ntInstance
      .getDoubleArrayTopic("/AdvantageKit/RealOutputs/" + VisionConstants.SIM_POSE_TOPIC_NAME)
      .getEntry(doubleArrayOf(0.0, 0.0, 0.0))

  val photonCameraSim = PhotonCamera(ntInstance, cameraName)

  val visionSim = SimVisionSystem("robot2023")

  init {
    visionSim.addCamera(
      PhotonCameraSim(photonCameraSim, CameraProperties.LL2_1280_720p),
      VisionConstants.CAMERA_TRANSFORMS[0].transform3d
    )

    visionSim.addVisionTargets(layout.apriltagFieldLayoutWPILIB)
  }

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    val dtPoseArr = dtPoseArraySub.get()
    visionSim.update(Pose2d(dtPoseArr[0], dtPoseArr[1], Rotation2d(dtPoseArr[2])))

    inputs.visionResult =
      VisionResult(
        photonCameraSim.latestResult.latencyMillis.milli.seconds,
        photonCameraSim.latestResult.timestampSeconds.seconds,
        photonCameraSim.latestResult.targets.map { VisionTarget(it) }
      )

    Logger.getInstance()
      .recordOutput("/Vision/visionSimPose", visionSim.getRobotPose(0.0).toPose2d())
  }
}
