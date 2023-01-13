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
package com.team4099.lib.utils.sim

import com.team4099.lib.vision.LogUtil.toPoseArray3d
import com.team4099.lib.utils.sim.PhotonCameraSim
import java.util.HashMap
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.HashSet
import java.util.function.BiConsumer
import java.util.stream.Collectors
import com.team4099.lib.vision.LogUtil
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import java.util.Optional
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.ArrayList
import java.util.List
import java.util.function.Consumer

class SimVisionSystem(visionSystemName: String) {
  private val tableName: String
  private val camSimMap: MutableMap<String, PhotonCameraSim?> = HashMap()
  private val camTrfMap: MutableMap<PhotonCameraSim?, Transform3d> = HashMap()
  private val robotPoseBuffer = TimeInterpolatableBuffer.createBuffer<Pose3d>(1.5)
  private val targetSets: MutableMap<String, MutableSet<SimVisionTarget>?> = HashMap()
  val debugField: Field2d

  /**
   * Get one of the simulated cameras.
   */
  fun getCameraSim(name: String): PhotonCameraSim? {
    return camSimMap[name]
  }

  /**
   * Get all of the simulated cameras.
   */
  val cameraSims: Collection<PhotonCameraSim?>
    get() = camSimMap.values

  /**
   * Get a simulated camera's position relative to the robot.
   */
  fun getRobotToCamera(name: String): Transform3d? {
    return camTrfMap[camSimMap[name]]
  }

  /**
   * Get a simulated camera's position relative to the robot.
   */
  fun getRobotToCamera(cameraSim: PhotonCameraSim?): Transform3d? {
    return camTrfMap[cameraSim]
  }

  /**
   * Adds a simulated camera to this vision system with a specified robot-to-camera transformation.
   * The vision targets registered with this vision system simulation will be observed by the simulated
   * [PhotonCamera].
   *
   * @param cameraSim The camera simulation
   * @param robotToCamera The transform from the robot pose to the camera pose
   */
  fun addCamera(cameraSim: PhotonCameraSim, robotToCamera: Transform3d) {
    val existing = camSimMap.putIfAbsent(cameraSim?.camera?.name!!, cameraSim)
    if (existing == null) {
      SmartDashboard.putData(
        tableName + "/" + cameraSim.camera.name + "/Sim Corners",
        cameraSim.debugCorners
      )
    }
    camTrfMap[cameraSim] = robotToCamera
  }

  /**
   * Remove all simulated cameras from this vision system.
   */
  fun clearCameras() {
    camSimMap.clear()
    camTrfMap.clear()
  }

  /**
   * Remove a simulated camera from this vision system.
   *
   * @return If the camera was present and removed
   */
  fun removeCamera(cameraSim: PhotonCameraSim): Boolean {
    val success = camSimMap.remove(cameraSim.camera.name) != null
    camTrfMap.remove(cameraSim)
    return success
  }

  /**
   * Adjust a camera's position relative to the robot. Use this if your camera is on a gimbal or
   * turret or some other mobile platform.
   *
   * @param cameraSim The simulated camera to change the relative position of
   * @param robotToCamera New transform from the robot to the camera
   */
  fun adjustCamera(cameraSim: PhotonCameraSim?, robotToCamera: Transform3d) {
    camTrfMap[cameraSim] = robotToCamera
  }

  val visionTargets: Set<SimVisionTarget>
    get() {
      val all = HashSet<SimVisionTarget>()
      for ((_, value) in targetSets) {
        all.addAll(value!!)
      }
      return all
    }

  fun getVisionTargets(type: String): Set<SimVisionTarget>? {
    return targetSets[type]
  }

  /**
   * Adds targets on the field which your vision system is designed to detect. The
   * [PhotonCamera]s simulated from this system will report the location of the camera
   * relative to the subset of these targets which are visible from the given camera position.
   *
   *
   * By default these are added under the type "targets".
   *
   * @param targets Targets to add to the simulated field
   */
  fun addVisionTargets(vararg targets: SimVisionTarget) {
    addVisionTargets("targets", *targets)
  }

  /**
   * Adds targets on the field which your vision system is designed to detect. The
   * [PhotonCamera]s simulated from this system will report the location of the camera
   * relative to the subset of these targets which are visible from the given camera position.
   *
   *
   * The AprilTags from this layout will be added as vision targets under the type "apriltags".
   * The poses added preserve the tag layout's current alliance origin.
   *
   * @param tagLayout The field tag layout to get Apriltag poses and IDs from
   */
  fun addVisionTargets(tagLayout: AprilTagFieldLayout) {
    for (tag in tagLayout.tags) {
      addVisionTargets(
        "apriltags",
        SimVisionTarget(
          tagLayout.getTagPose(tag.ID).get(),  // preserve alliance rotation
          Units.inchesToMeters(6.0),
          tag.ID
        )
      )
    }
  }

  /**
   * Adds targets on the field which your vision system is designed to detect. The
   * [PhotonCamera]s simulated from this system will report the location of the camera
   * relative to the subset of these targets which are visible from the given camera position.
   *
   * @param type Type of target (e.g. "cargo").
   * @param targets Targets to add to the simulated field
   */
  fun addVisionTargets(type: String, vararg targets: SimVisionTarget) {
    if (targetSets[type] == null) targetSets[type] = HashSet()
    for (tgt in targets) {
      targetSets[type]!!.add(tgt)
    }
  }

  fun clearVisionTargets() {
    targetSets.clear()
  }

  fun removeVisionTargets(type: String): Set<SimVisionTarget>? {
    return targetSets.remove(type)
  }

  fun removeVisionTargets(vararg targets: SimVisionTarget?): Set<SimVisionTarget> {
    val removeList = List.of(*targets)
    val removedSet = HashSet<SimVisionTarget>()
    for ((_, value) in targetSets) {
      value!!.removeIf { t: SimVisionTarget ->
        if (removeList.contains(t)) {
          removedSet.add(t)
          return@removeIf true
        } else return@removeIf false
      }
    }
    return removedSet
  }

  fun clearRobotPoses() {
    robotPoseBuffer.clear()
    camSimMap.forEach { (name: String?, cam: PhotonCameraSim?) -> cam!!.clearCameraPoses() }
  }

  /**
   * Get the robot pose in meters saved by the vision system secondsAgo.
   * @param secondsAgo Seconds to look into the past
   */
  fun getRobotPose(secondsAgo: Double): Pose3d {
    return robotPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(Pose3d())
  }

  /**
   * Periodic update. Ensure this is called repeatedly-- camera performance is used to
   * automatically determine if a new frame should be submitted.
   * @param robotPoseMeters The current robot pose in meters
   */
  fun update(robotPoseMeters: Pose2d) {
    val pose3d = Pose3d(robotPoseMeters.x, robotPoseMeters.y, 0.0, Rotation3d(0.0, 0.0, robotPoseMeters.rotation.radians))
    update(pose3d)
  }

  /**
   * Periodic update. Ensure this is called repeatedly-- camera performance is used to
   * automatically determine if a new frame should be submitted.
   * @param robotPoseMeters The current robot pose in meters
   */
  fun update(robotPoseMeters: Pose3d?) {
    val targetTypes: Set<Map.Entry<String, Set<SimVisionTarget>?>> = targetSets.entries
    // update vision targets on field
    targetTypes.forEach(Consumer { (key, value): Map.Entry<String, Set<SimVisionTarget>?> ->
      debugField.getObject(
        key
      ).poses = value!!.stream().map { t: SimVisionTarget -> t.pose.toPose2d() }
        .collect(Collectors.toList())
    })
    if (robotPoseMeters == null) return

    // save "real" robot poses over time
    val now = Timer.getFPGATimestamp()
    robotPoseBuffer.addSample(now, robotPoseMeters)
    debugField.robotPose = robotPoseMeters.toPose2d()
    SmartDashboard.putNumberArray(
      "$tableName/RobotPose3d",
      toPoseArray3d(robotPoseMeters)
    )
    val allTargets = ArrayList<SimVisionTarget>()
    targetTypes.forEach(Consumer { (_, value): Map.Entry<String, Set<SimVisionTarget>?> ->
      allTargets.addAll(
        value!!
      )
    })
    val visibleTargets = ArrayList<Pose3d>()
    val cameraPose2ds = ArrayList<Pose2d>()
    // process each camera
    for (camSim in camSimMap.values) {
      // check if this camera is ready to process and get latency
      val optionalLatency = camSim!!.shouldProcess
      if (optionalLatency.isEmpty) continue
      val latencyMillis = optionalLatency.get()
      // save "real" camera poses over time
      camSim.updateCameraPose(robotPoseMeters.transformBy(getRobotToCamera(camSim)))
      // display camera latency milliseconds ago
      val lateCameraPose = camSim.getCameraPose(latencyMillis / 1000.0)
      cameraPose2ds.add(lateCameraPose.toPose2d())
      SmartDashboard.putNumberArray(
        tableName + "/" + camSim.camera.name + "/LatePose3d",
        toPoseArray3d(lateCameraPose)
      )

      // update camera's visible targets
      val camResult = camSim.process(latencyMillis, lateCameraPose, allTargets)
      // display results
      for (target in camResult.getTargets()) {
        visibleTargets.add(
          lateCameraPose.transformBy(target.bestCameraToTarget)
        )
      }
    }
    if (visibleTargets.size != 0) {
      SmartDashboard.putNumberArray("$tableName/EstTargetPoses3d", toPoseArray3d(visibleTargets))
      debugField.getObject("visibleTargets").poses = visibleTargets.stream().map { p: Pose3d -> p.toPose2d() }
        .collect(Collectors.toList())
    }
    if (cameraPose2ds.size != 0) debugField.getObject("cameras").poses = cameraPose2ds
  }

  /**
   * Create a simulated vision system involving a camera(s) and coprocessor(s) mounted on a mobile robot
   * running PhotonVision, detecting one or more targets scattered around the field.
   */
  init {
    debugField = Field2d()
    tableName = "vision-$visionSystemName"
    SmartDashboard.putData("$tableName/Sim Field", debugField)
  }
}
