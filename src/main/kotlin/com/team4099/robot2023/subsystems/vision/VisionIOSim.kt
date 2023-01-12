package com.team4099.robot2023.subsystems.vision

import com.team4099.apriltag.AprilTagFieldLayout
import com.team4099.lib.utils.estimation.CameraProperties
import com.team4099.lib.utils.sim.PhotonCamera
import com.team4099.lib.utils.sim.PhotonCameraSim
import com.team4099.lib.utils.sim.SimVisionSystem
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.VisionConstants
import edu.wpi.first.networktables.NetworkTableInstance
import org.photonvision.targeting.PhotonPipelineResult
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians

object VisionIOSim : VisionIO {
  val layout: AprilTagFieldLayout =
    AprilTagFieldLayout(
      FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth
    )

  val ntInstance = NetworkTableInstance.getDefault()
  val dtPoseArraySub =
    ntInstance
      .getDoubleArrayTopic("RealOutputs/" + VisionConstants.POSE_TOPIC_NAME)
      .getEntry(doubleArrayOf(0.0, 0.0, 0.0))

  val frontCamera = PhotonCamera(ntInstance, VisionConstants.FRONT_CAMERA_NAME)
  val sideCamera = PhotonCamera(ntInstance, VisionConstants.SIDE_CAMERA_NAME)
  val backCamera = PhotonCamera(ntInstance, VisionConstants.BACK_CAMERA_NAME)

  val cameras = listOf(frontCamera) // TODO add more cameras
  val results = listOf(PhotonPipelineResult()) // add one for each camera

  val visionSim = SimVisionSystem("robot2023")

  init {
    // todo add multiple cameras

    visionSim.addCamera(
      PhotonCameraSim(frontCamera, CameraProperties.PI4_PICAM2_480p),
      VisionConstants.SIM_CAMERA_TRANSFORMS[0].transform3d
    )

    visionSim.addVisionTargets(layout.apriltagFieldLayoutWPILIB)
  }

  override fun updateInputs(inputs: VisionIO.VisionInputs) {
    val dtPoseArr = dtPoseArraySub.get()
    visionSim.update(Pose2d(dtPoseArr[0].meters, dtPoseArr[1].meters, dtPoseArr[2].radians).pose2d)
    inputs.photonResults = cameras.map { it.latestResult }
  }
}
