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
package com.team4099.lib.utils.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

object PhotonUtils {
  /**
   * Returns the yaw between the robot pose and target translation.
   *
   * @param robotPose Current pose of the robot
   * @param targetTranslation Translation of the target on the field
   * @return Yaw to the target
   */
  fun getYawToTarget(robotPose: Pose2d?, targetTranslation: Translation2d?): Rotation2d {
    return getYawToTarget(robotPose, Pose2d(targetTranslation, Rotation2d()))
  }

  /**
   * Returns the yaw between the robot pose and target translation.
   *
   * @param robotPose Current pose of the robot
   * @param targetPose Pose of the target on the field
   * @return Yaw to the target
   */
  fun getYawToTarget(robotPose: Pose2d?, targetPose: Pose2d): Rotation2d {
    return targetPose.relativeTo(robotPose).translation.angle
  }

  /**
   * Estimates the pose of the robot in the field coordinate system, given the position of the
   * target relative to the camera, the target relative to the field, and the camera relative to the
   * camera.
   *
   * @param fieldTarget The position of the target in the field.
   * @param cameraToTarget The transform from the camera to the target.
   * @param robotToCamera The position of the camera relative to the robot. If the camera was
   * mounted 3 inches behind and 30 inches above the "origin" (usually bottom-center) of the robot,
   * the translation component would be [-3 inches, 0 inches, 30 inches]. If the camera was pitched
   * up 20 degrees (with "0" pitch being perpendicular to the robot), the rotation component would
   * be [0 degrees, -20 degrees, 0 degrees].
   * @return The position of the robot in the field.
   */
  fun estimateFieldToRobot(
    fieldTarget: Pose3d, cameraToTarget: Transform3d, robotToCamera: Transform3d
  ): Pose3d {
    return estimateFieldToCamera(fieldTarget, cameraToTarget).plus(robotToCamera.inverse())
  }

  /**
   * Estimates the pose of the camera in the field coordinate system, given the position of the
   * target relative to the camera, and the target relative to the field. This only tracks the
   * position of the camera, not the position of the robot itself.
   *
   * @param fieldToTarget The position of the target in the field.
   * @param cameraToTarget The transform from the camera to the target.
   * @return The position of the camera in the field.
   */
  fun estimateFieldToCamera(fieldToTarget: Pose3d, cameraToTarget: Transform3d): Pose3d {
    return fieldToTarget.transformBy(cameraToTarget.inverse())
  }

  /**
   * Estimates a [Transform3d] that maps the camera position to the target position, using the
   * robot's yaw angle(likely from odometry). Note that this transformation's rotation maps onto the rotation
   * of the target's pose, and any inaccuracies in the given robot angle will heavily alter the
   * robot pose that may be estimated from the target with this transform.
   *
   * @param cameraToTargetTranslation A Translation3d that describes the [x,y,z] position of the target
   * relative to the camera.
   * @param fieldToTarget A Pose3d representing the target position in the field coordinate system.
   * @param robotAngle The current robot yaw angle, likely from odometry.
   * @param robotToCamera The position of the camera relative to the robot. If the camera was
   * mounted 3 inches behind and 30 inches above the "origin" (usually bottom-center) of the robot,
   * the translation component would be [-3 inches, 0 inches, 30 inches]. If the camera was pitched
   * up 20 degrees (with "0" pitch being perpendicular to the robot), the rotation component would
   * be [0 degrees, -20 degrees, 0 degrees].
   * @return A Transform3d that brings the camera pose to the target pose
   */
  fun estimateCameraToTarget(
    cameraToTargetTranslation: Translation3d?, fieldToTarget: Pose3d,
    robotAngle: Rotation2d, robotToCamera: Transform3d
  ): Transform3d {
    // find the rotation of our camera given its orientation relative to the robot angle
    val cameraRot = Rotation3d(0.0, 0.0, robotAngle.radians).plus(robotToCamera.rotation)
    return Transform3d(
      cameraToTargetTranslation,
      fieldToTarget.rotation.minus(cameraRot)
    )
  }

  /**
   * Find the range(distance along ground) from the camera to the target through trigonometry
   * using the known target and camera heights. This method assumes the robot height and
   * pitch/roll to be zero.
   *
   *
   * Note: The accuracy of this estimation depends on the difference in height between the camera
   * and the target-- as the difference in height shrinks, the inaccuracy grows. Targets which are
   * at the same height as the camera can not be estimated and will return zero.
   *
   * @param robotToCamera The position of the camera relative to the robot. If the camera was
   * mounted 3 inches behind and 30 inches above the "origin" (usually bottom-center) of the robot,
   * the translation component would be [-3 inches, 0 inches, 30 inches]. If the camera was pitched
   * up 20 degrees (with "0" pitch being perpendicular to the robot), the rotation component would
   * be [0 degrees, -20 degrees, 0 degrees].
   * @param targetHeight The known height of the observed target in the world.
   * @param targetYaw The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values left.
   * @param targetPitch The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values down.
   * @return The range(distance along ground) from the camera to the target
   */
  fun estimateCamToTargetRange(
    robotToCamera: Transform3d?,
    targetHeight: Double, targetYaw: Rotation2d, targetPitch: Rotation2d
  ): Double {
    return estimateCamToTargetTrl(robotToCamera, targetHeight, targetYaw, targetPitch)
      .toTranslation2d()
      .norm
  }

  /**
   * Find the 3d camera-to-target translation(the target translation relative to the camera) through
   * trigonometry using the known target and camera heights. This method assumes the robot height
   * and pitch/roll to be zero.
   *
   *
   * Note: The accuracy of this estimation depends on the difference in height between the camera
   * and the target-- as the difference in height shrinks, the inaccuracy grows. Targets which are
   * at the same height as the camera can not be estimated and will return zero.
   *
   * @param robotToCamera The position of the camera relative to the robot. If the camera was
   * mounted 3 inches behind and 30 inches above the "origin" (usually bottom-center) of the robot,
   * the translation component would be [-3 inches, 0 inches, 30 inches]. If the camera was pitched
   * up 20 degrees (with "0" pitch being perpendicular to the robot), the rotation component would
   * be [0 degrees, -20 degrees, 0 degrees].
   * @param targetHeight The known height of the observed target in the world.
   * @param targetYaw The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values left.
   * @param targetPitch The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values down.
   * @return The target's translation relative to the camera
   */
  fun estimateCamToTargetTrl(
    robotToCamera: Transform3d?,
    targetHeight: Double, targetYaw: Rotation2d, targetPitch: Rotation2d
  ): Translation3d {
    return estimateCamToTargetTrl(
      0.0, Rotation3d(), robotToCamera,
      targetHeight, targetYaw, targetPitch
    )
  }

  /**
   * Find the 3d camera-to-target translation(the target translation relative to the camera) through
   * trigonometry using the known target and camera heights.
   *
   *
   * Note: The accuracy of this estimation depends on the difference in height between the camera
   * and the target-- as the difference in height shrinks, the inaccuracy grows. Targets which are
   * at the same height as the camera can not be estimated and will return zero.
   *
   * @param robotHeight The height(z) of the robot's pose off the ground. The 3d robot pose
   * is usually measured as the bottom-center of the drivetrain with height(z) == 0,
   * unless on top of some element which elevates the robot.
   * @param robotPoseRot The 3d rotation of the robot, likely from a gyro or accelerometer.
   * This is also usually zero [0, 0, 0]. The yaw of this rotation does not matter,
   * only the roll and pitch.
   * @param robotToCamera The position of the camera relative to the robot. If the camera was
   * mounted 3 inches behind and 30 inches above the "origin" (usually bottom-center) of the robot,
   * the translation component would be [-3 inches, 0 inches, 30 inches]. If the camera was pitched
   * up 20 degrees (with "0" pitch being perpendicular to the robot), the rotation component would
   * be [0 degrees, -20 degrees, 0 degrees].
   * @param targetHeight The known height of the observed target in the world.
   * @param targetYaw The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values left.
   * @param targetPitch The yaw of the target in the camera's 2d image, calculated from the camera's FOV.
   * Positive values down.
   * @return The target's translation relative to the camera
   */
  fun estimateCamToTargetTrl(
    robotHeight: Double, robotPoseRot: Rotation3d?, robotToCamera: Transform3d?,
    targetHeight: Double, targetYaw: Rotation2d, targetPitch: Rotation2d
  ): Translation3d {
    // we measure the difference in height, so find the camera's height in the world
    val robotPose = Pose3d(0.0, 0.0, robotHeight, robotPoseRot)
    val camPose = robotPose.plus(robotToCamera)
    val camHeight = camPose.z
    val heightDiff = targetHeight - camHeight
    if (Math.abs(heightDiff) < 1e-5) return Translation3d()
    // convert 2d target point in camera image to 3d point on a plane 1 meter away
    var camToTargTrl = Translation3d(
      1.0,
      Math.tan(targetYaw.radians),
      -Math.tan(targetPitch.radians)
    )
    // normalize this translation to be unit-length distance from the origin(camera)
    // (point a 3d plane converted to point a 3d unit sphere using the intersecting line)
    camToTargTrl = camToTargTrl.div(camToTargTrl.norm)
    // non-zero camera orientations rotate this translation
    camToTargTrl = camToTargTrl.rotateBy(camPose.rotation)
    // we then scale this translation so the target point z matches its known height
    camToTargTrl = camToTargTrl.times(heightDiff / camToTargTrl.z)
    // this gives us the correct target translation relative to the camera's translation
    // we want this object relative to the camera's pose (include rotation) instead
    camToTargTrl = Pose3d(camToTargTrl, Rotation3d())
      .relativeTo(Pose3d(Translation3d(), camPose.rotation))
      .translation
    return camToTargTrl
  }
}
