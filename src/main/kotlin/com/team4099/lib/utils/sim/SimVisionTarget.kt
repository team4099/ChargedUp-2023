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

import com.team4099.lib.utils.estimation.TargetModel
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation3d
import kotlin.jvm.JvmOverloads

class SimVisionTarget {
  var pose: Pose3d
  var model: TargetModel
  @JvmField
  val id: Int

  /**
   * Describes a vision target located somewhere on the field that your SimVisionSystem can detect.
   *
   * @param pose Pose3d of the tag in field-relative coordinates
   * @param model TargetModel which describes the shape of the target
   */
  constructor(pose: Pose3d, model: TargetModel) {
    this.pose = pose
    this.model = model
    id = -1
  }

  /**
   * Describes an orientation-agnostic vision target (like a sphere) located somewhere on the field
   * that your SimVisionSystem can detect.
   *
   * @param pose Pose3d of the target in field-relative coordinates
   * @param diamMeters Size(diameter) of the outer bounding box of the target in meters.
   */
  constructor(pose: Pose3d, diamMeters: Double) : this(pose, diamMeters, diamMeters, -1) {}

  /**
   * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
   *
   * @param pose Pose3d of the tag in field-relative coordinates
   * @param lengthMeters Width/height of the outer bounding box of the tag(black square) in meters.
   * @param id The ID of this fiducial tag
   */
  constructor(pose: Pose3d, lengthMeters: Double, id: Int) : this(pose, lengthMeters, lengthMeters, id) {}
  /**
   * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
   *
   * @param pose Pose3d of the tag in field-relative coordinates
   * @param widthMeters Width of the outer bounding box of the tag(black square) in meters.
   * @param heightMeters Height of the outer bounding box of the tag(black square) in meters.
   * @param id The ID of this fiducial tag
   */
  /**
   * Describes a planar vision target located somewhere on the field that your SimVisionSystem can detect.
   *
   * @param pose Pose3d of the target in field-relative coordinates
   * @param widthMeters Width of the outer bounding box of the target in meters.
   * @param heightMeters Height of the outer bounding box of the target in meters.
   */
  @JvmOverloads
  constructor(pose: Pose3d, widthMeters: Double, heightMeters: Double, id: Int = -1) {
    this.pose = pose
    model = TargetModel.ofPlanarRect(widthMeters, heightMeters)
    this.id = id
  }

  val planarFieldCorners: List<Translation3d>
    get() = model.getFieldCorners(pose)

  override fun equals(obj: Any?): Boolean {
    if (this === obj) return true
    if (obj is SimVisionTarget) {
      val o = obj
      return pose == o.pose && model == o.model
    }
    return false
  }
}
