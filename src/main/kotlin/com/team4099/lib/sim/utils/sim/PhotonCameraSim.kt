/*
* MIT License
*
* Copyright (c) 2022 PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
package com.team4099.lib.sim.utils.sim

import com.team4099.lib.sim.utils.util.VideoSimUtil.updateVideoProp
import com.team4099.lib.sim.utils.util.VideoSimUtil.warp16h5TagImage
import com.team4099.lib.sim.utils.util.VideoSimUtil.drawTagDetection
import kotlin.jvm.JvmOverloads
import com.team4099.lib.sim.utils.estimation.CameraProperties
import edu.wpi.first.networktables.NetworkTableEntry
import org.photonvision.PhotonTargetSortMode
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.cscore.CvSource
import org.opencv.core.Mat
import edu.wpi.first.cscore.CameraServerCvJNI
import java.lang.RuntimeException
import com.team4099.lib.sim.utils.util.CameraTargetRelation
import org.photonvision.targeting.TargetCorner
import java.util.Optional
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.TreeMap
import org.opencv.core.CvType
import com.team4099.lib.sim.utils.estimation.OpenCVHelp
import com.team4099.lib.sim.utils.estimation.VisionEstimation.PNPResults
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy
import org.opencv.imgproc.Imgproc
import edu.wpi.first.math.geometry.Rotation2d
import java.util.stream.Collectors
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.VideoMode.PixelFormat
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Timer
import org.opencv.core.Size
import org.photonvision.PhotonVersion
import org.photonvision.common.dataflow.structures.Packet
import java.lang.Exception
import java.util.ArrayList

class PhotonCameraSim @JvmOverloads constructor(
  val camera: PhotonCamera,
  /**
   * This simulated camera's [CameraProperties]
   */
  val prop: CameraProperties = CameraProperties.PERFECT_90DEG
) {
  private val latencyMillisEntry: NetworkTableEntry
  private val hasTargetEntry: NetworkTableEntry
  private val targetPitchEntry: NetworkTableEntry
  private val targetYawEntry: NetworkTableEntry
  private val targetAreaEntry: NetworkTableEntry
  private val targetSkewEntry: NetworkTableEntry
  private val targetPoseEntry: NetworkTableEntry
  private val versionEntry: NetworkTableEntry
  private var lastTime = Timer.getFPGATimestamp()
  private var msUntilNextFrame = 0.0
  var maxSightRangeMeters = Double.MAX_VALUE
    private set

  /**
   * The minimum percentage(0 - 100) a detected target must take up of the camera's image
   * to be processed.
   */
  var minTargetAreaPercent = 0.0
  private var sortMode: PhotonTargetSortMode? = PhotonTargetSortMode.Largest
  private val camPoseBuffer = TimeInterpolatableBuffer.createBuffer<Pose3d>(1.5)
  val debugCorners = Field2d()

  // video stream simulation
  val videoSimRaw: CvSource
  private val videoSimFrameRaw = Mat()
  private var videoSimRawEnabled = true
  private val videoSimProcessed: CvSource
  private val videoSimFrameProcessed = Mat()
  private var videoSimProcEnabled = true

  companion object {
    private const val kDefaultMinAreaPx = 100.0

    init {
      try {
        CameraServerCvJNI.forceLoad()
      } catch (e: Exception) {
        throw RuntimeException("Failed to load native libraries!", e)
      }
    }
  }

  /**
   * Constructs a handle for simulating [PhotonCamera] values.
   * Processing simulated targets through this class will change the associated
   * PhotonCamera's results.
   *
   * @param camera The camera to be simulated
   * @param prop Properties of this camera such as FOV and FPS
   * @param minTargetAreaPercent The minimum percentage(0 - 100) a detected target must take up of the
   * camera's image to be processed. Match this with your contour filtering settings in the
   * PhotonVision GUI.
   * @param maxSightRangeMeters Maximum distance at which the target is illuminated to your camera.
   * Note that minimum target area of the image is separate from this.
   */
  constructor(
    camera: PhotonCamera, prop: CameraProperties,
    minTargetAreaPercent: Double, maxSightRangeMeters: Double
  ) : this(camera, prop) {
    this.minTargetAreaPercent = minTargetAreaPercent
    this.maxSightRangeMeters = maxSightRangeMeters
  }

  /**
   * Get the camera pose in meters saved by the vision system secondsAgo.
   * @param secondsAgo Seconds to look into the past
   */
  fun getCameraPose(secondsAgo: Double): Pose3d {
    return camPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(Pose3d())
  }

  /**
   * The minimum number of pixels a detected target must take up in the camera's image
   * to be processed.
   */
  var minTargetAreaPixels: Double
    get() = minTargetAreaPercent / 100.0 * prop.resArea
    set(areaPx) {
      minTargetAreaPercent = areaPx / prop.resArea * 100
    }

  /**
   * Defines the order the targets are sorted in the pipeline result.
   */
  var targetSortMode: PhotonTargetSortMode?
    get() = sortMode
    set(sortMode) {
      if (sortMode != null) this.sortMode = sortMode
    }

  /**
   * Determines if this target's pose should be visible to the camera without considering
   * its projected image points. Does not account for image area.
   * @param camPose Camera's 3d pose
   * @param target Vision target containing pose and shape
   * @return If this vision target can be seen before image projection.
   */
  fun canSeeTargetPose(
    camPose: Pose3d?,
    target: SimVisionTarget
  ): Boolean {
    val rel = CameraTargetRelation(camPose!!, target.pose)
    // target translation is outside of camera's FOV
    return (Math.abs(rel.camToTargYaw.degrees) < (prop.horizFOV.degrees / 2)) &&
      Math.abs(rel.camToTargPitch.degrees) < prop.vertFOV.degrees / 2 &&  // camera is behind planar target and it should be occluded
      (!target.model.isPlanar || Math.abs(rel.targToCamAngle.degrees) < 90) &&  // target is too far
      rel.camToTarg.translation.norm <= maxSightRangeMeters
  }

  /**
   * Determines if all target corners are inside the camera's image.
   * @param corners The corners of the target as image points(x,y)
   */
  fun canSeeCorners(corners: List<TargetCorner>): Boolean {
    // corner is outside of resolution
    for (corner in corners) {
      if (MathUtil.clamp(corner.x, 0.0, prop.resWidth.toDouble()) != corner.x ||
        MathUtil.clamp(corner.y, 0.0, prop.resHeight.toDouble()) != corner.y
      ) {
        return false
      }
    }
    return true
  }// this camera isnt ready to process yet// check if this camera is ready for another frame update

  /**
   * Determine if this camera should process a new frame based on performance metrics and the time
   * since the last update. This returns an Optional which is either empty if no update should occur
   * or a Double of the latency in milliseconds of the frame which should be processed. If a
   * latency is returned, the last frame update time becomes the current time.
   * @return Optional double which is empty while blocked or the latency in milliseconds if ready
   */
  val shouldProcess: Optional<Double>
    get() {
      // check if this camera is ready for another frame update
      val now = Timer.getFPGATimestamp()
      val dt = now - lastTime
      val latencyMillis: Double
      return if (dt >= msUntilNextFrame / 1000.0) {
        latencyMillis = prop.estLatencyMs()
        msUntilNextFrame = prop.estMsUntilNextFrame()
        lastTime = now
        Optional.of(latencyMillis)
      } else {
        // this camera isnt ready to process yet
        Optional.empty()
      }
    }

  /**
   * Maximum distance at which the target is illuminated to your camera.
   * Note that minimum target area of the image is separate from this.
   */
  fun setMaxSightRange(rangeMeters: Double) {
    maxSightRangeMeters = rangeMeters
  }

  /**
   * Sets whether the raw video stream simulation is enabled.
   */
  fun enableRawStream(enabled: Boolean) {
    videoSimRawEnabled = enabled
  }

  /**
   * Sets whether the processed video stream simulation is enabled.
   */
  fun enableProcessedStream(enabled: Boolean) {
    videoSimProcEnabled = enabled
  }

  /**
   * Update the current camera pose given the current robot pose in meters.
   * This is dependent on the robot-to-camera transform, and camera poses are saved over time.
   */
  fun updateCameraPose(cameraPose: Pose3d) {
    camPoseBuffer.addSample(Timer.getFPGATimestamp(), cameraPose)
  }

  fun clearCameraPoses() {
    camPoseBuffer.clear()
  }

  fun process(
    latencyMillis: Double, cameraPose: Pose3d, targets: List<SimVisionTarget>
  ): PhotonPipelineResult {
    val detectableTgts = ArrayList<PhotonTrackedTarget>()
    val visibleTgts = TreeMap<Double, Pair<Int, List<TargetCorner>>>()
    val dbgVisCorners = ArrayList<TargetCorner>()
    val dbgBestCorners = ArrayList<TargetCorner>()

    // reset our frame
    updateVideoProp(videoSimRaw, prop)
    updateVideoProp(videoSimProcessed, prop)
    val videoFrameSize = Size(
      prop.resWidth.toDouble(), prop.resHeight.toDouble()
    )
    Mat.zeros(videoFrameSize, CvType.CV_8UC1).assignTo(videoSimFrameRaw)
    for (tgt in targets) {
      // pose isn't visible, skip to next
      if (!canSeeTargetPose(cameraPose, tgt)) continue

      // find target's 3d corner points
      var fieldCorners = tgt.model.getFieldCorners(tgt.pose)
      if (!tgt.model.isPlanar) {
        fieldCorners = tgt.model.getAgnosticFieldCorners(cameraPose, tgt.pose)
      }
      // project 3d target points into 2d image points
      val targetCorners = OpenCVHelp.projectPoints(
        cameraPose,
        prop,
        fieldCorners
      )
      // save visible tags sorted on distance for stream simulation
      if (tgt.id >= 0) {
        visibleTgts[tgt.pose.translation.getDistance(cameraPose.translation)] = Pair(tgt.id, targetCorners)
      }
      // estimate pixel noise
      val noisyTargetCorners = prop.estPixelNoise(targetCorners)
      // find the 2d yaw/pitch
      val boundingCenterRot = prop.getPixelRot(noisyTargetCorners)
      // find contour area
      val areaPercent = prop.getContourAreaPercent(noisyTargetCorners)

      // projected target can't be detected, skip to next
      if (!(canSeeCorners(noisyTargetCorners) && areaPercent >= minTargetAreaPercent)) continue

      // only do 3d estimation if we have a planar target
      var pnpSim = PNPResults()
      if (tgt.model.isPlanar) {
        pnpSim = OpenCVHelp.solveTagPNP(prop, tgt.model.cornerOffsets, noisyTargetCorners)
      }
      detectableTgts.add(
        PhotonTrackedTarget(
          Math.toDegrees(boundingCenterRot.z),
          -Math.toDegrees(boundingCenterRot.y),
          areaPercent,
          Math.toDegrees(boundingCenterRot.x),
          tgt.id,
          pnpSim.best,
          pnpSim.alt,
          pnpSim.ambiguity,
          noisyTargetCorners,
          targetCorners
        )
      )
      dbgVisCorners.addAll(noisyTargetCorners)
      if (dbgBestCorners.size == 0) dbgBestCorners.addAll(noisyTargetCorners)
    }
    // render visible tags to raw video frame
    if (videoSimRawEnabled) {
      for (detect in visibleTgts.descendingMap().values) {
        warp16h5TagImage(
          detect.first,
          OpenCVHelp.targetCornersToMat(detect.second),
          videoSimFrameRaw, true
        )
      }
      videoSimRaw.putFrame(videoSimFrameRaw)
    } else videoSimRaw.setConnectionStrategy(ConnectionStrategy.kForceClose)
    // draw/annotate tag detection outline on processed view
    if (videoSimProcEnabled) {
      Imgproc.cvtColor(videoSimFrameRaw, videoSimFrameProcessed, Imgproc.COLOR_GRAY2BGR)
      for (tag in detectableTgts) {
        drawTagDetection(
          tag.fiducialId,
          OpenCVHelp.targetCornersToMat(tag.detectedCorners),
          videoSimFrameProcessed
        )
      }
      videoSimProcessed.putFrame(videoSimFrameProcessed)
    } else videoSimProcessed.setConnectionStrategy(ConnectionStrategy.kForceClose)
    debugCorners.getObject("corners").poses = prop.getPixelFraction(dbgVisCorners)
      .stream()
      .map { p: TargetCorner -> Pose2d(p.x, 1 - p.y, Rotation2d()) }
      .collect(Collectors.toList())
    debugCorners.getObject("bestCorners").poses = prop.getPixelFraction(dbgBestCorners)
      .stream()
      .map { p: TargetCorner -> Pose2d(p.x, 1 - p.y, Rotation2d()) }
      .collect(Collectors.toList())
    debugCorners.getObject("aspectRatio").poses = prop.getPixelFraction(
      java.util.List.of(
        TargetCorner(0.0, 0.0),
        TargetCorner(prop.resWidth.toDouble(), 0.0),
        TargetCorner(prop.resWidth.toDouble(), prop.resHeight.toDouble()),
        TargetCorner(0.0, prop.resHeight.toDouble())
      )
    ).stream()
      .map { p: TargetCorner -> Pose2d(p.x, 1 - p.y, Rotation2d()) }
      .collect(Collectors.toList())

    // put this simulated data to NT
    if (sortMode != null) {
      detectableTgts.sortWith(sortMode!!.comparator)
    }
    val result = PhotonPipelineResult(latencyMillis, detectableTgts)
    submitProcessedFrame(result)
    return result
  }

  /**
   * Simulate one processed frame of vision data, putting one result to NT.
   *
   * @param result The pipeline result to submit
   */
  fun submitProcessedFrame(result: PhotonPipelineResult) {
    latencyMillisEntry.setDouble(result.latencyMillis)
    val newPacket = Packet(result.packetSize)
    result.populatePacket(newPacket)
    camera.rawBytesEntry.setRaw(newPacket.data)
    val hasTargets = result.hasTargets()
    hasTargetEntry.setBoolean(hasTargets)
    if (!hasTargets) {
      targetPitchEntry.setDouble(0.0)
      targetYawEntry.setDouble(0.0)
      targetAreaEntry.setDouble(0.0)
      targetPoseEntry.setDoubleArray(doubleArrayOf(0.0, 0.0, 0.0))
      targetSkewEntry.setDouble(0.0)
    } else {
      val bestTarget = result.bestTarget
      targetPitchEntry.setDouble(bestTarget.pitch)
      targetYawEntry.setDouble(bestTarget.yaw)
      targetAreaEntry.setDouble(bestTarget.area)
      targetSkewEntry.setDouble(bestTarget.skew)
      val transform = bestTarget.bestCameraToTarget
      val poseData = doubleArrayOf(
        transform.x, transform.y, transform.rotation.toRotation2d().degrees
      )
      targetPoseEntry.setDoubleArray(poseData)
    }
  }
  /**
   * Constructs a handle for simulating [PhotonCamera] values.
   * Processing simulated targets through this class will change the associated
   * PhotonCamera's results.
   *
   *
   * By default, the minimum target area is 100 pixels and there is no maximum sight range.
   *
   * @param camera The camera to be simulated
   * @param prop Properties of this camera such as FOV and FPS
   */
  /**
   * Constructs a handle for simulating [PhotonCamera] values.
   * Processing simulated targets through this class will change the associated
   * PhotonCamera's results.
   *
   *
   * **This constructor's camera has a 90 deg FOV with no simulated lag!**
   *
   *
   * By default, the minimum target area is 100 pixels and there is no maximum sight range.
   *
   * @param camera The camera to be simulated
   */
  init {
    minTargetAreaPixels = kDefaultMinAreaPx
    videoSimRaw = CameraServer.putVideo(
      camera.name + "-raw", prop.resWidth, prop.resHeight
    )
    videoSimRaw.setPixelFormat(PixelFormat.kGray)
    videoSimProcessed = CameraServer.putVideo(
      camera.name + "-processed", prop.resWidth, prop.resHeight
    )
    val rootTable = camera.rootTable
    latencyMillisEntry = rootTable.getEntry("latencyMillis")
    hasTargetEntry = rootTable.getEntry("hasTargetEntry")
    targetPitchEntry = rootTable.getEntry("targetPitchEntry")
    targetYawEntry = rootTable.getEntry("targetYawEntry")
    targetAreaEntry = rootTable.getEntry("targetAreaEntry")
    targetSkewEntry = rootTable.getEntry("targetSkewEntry")
    targetPoseEntry = rootTable.getEntry("targetPoseEntry")
    versionEntry = camera.versionEntry
    // Sets the version string so that it will always match the current version
    versionEntry.setString(PhotonVersion.versionString)
  }
}
