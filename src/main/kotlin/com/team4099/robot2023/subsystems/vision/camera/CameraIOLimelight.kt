package com.team4099.robot2023.subsystems.vision.camera

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import com.team4099.lib.vision.TargetCorner
import com.team4099.lib.vision.VisionResult
import com.team4099.lib.vision.VisionTarget
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance

class CameraIOLimelight(
  override val transformToRobot: Transform3d,
  override val cameraName: String
) : CameraIO {
  val jsonType = object : TypeToken<Map<String, Map<String, Any>>>() {}.type

  val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")
  val limelightJsonSub = limelightTable.getStringTopic("json").getEntry("[]")
  val latencySub =
    limelightTable
      .getDoubleTopic("tl")
      .getEntry(0.0) // add at least 11 ms for image capture latency

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    val limelightJson =
      Gson().fromJson<Map<String, Map<String, Any>>>(limelightJsonSub.get(), jsonType)
    val results = limelightJson["Results"]
    val fiducials: List<Map<String, Any>> =
      (results?.get("Fiducial") ?: listOf<Map<String, Any>>()) as List<HashMap<String, Any>>

    val trackedTargets =
      fiducials.map {
        VisionTarget(
          it["tx"].toString().toDouble(),
          it["ty"].toString().toDouble(),
          it["ta"].toString().toDouble(),
          it["fID"].toString().toDouble().toInt(),
          targetSpaceArrayToTransform(stringToDoubleArray(it["t6r_ts"].toString())),
          coordinateArraytoTargetCorners(ptsToDoubleArray(it["pts"].toString()))
        )
      }

    // not sure if we want fpga timestamp here
    val latencyResult = latencySub.get() * 1E-3 + 11 * 1E-3
    inputs.visionResult =
      VisionResult(
        latencyResult, limelightJsonSub.lastChange * 1E-6 - latencyResult, trackedTargets
      )
  }

  private fun targetSpaceArrayToTransform(array: Array<Double>): Transform3d {
    // https://docs.limelightvision.io/en/latest/coordinate_systems_fiducials.html#target-space
    return Transform3d(
      Translation3d(array[2], array[0], -array[1]),
      Rotation3d(
        Units.degreesToRadians(array[3]),
        Units.degreesToRadians(array[5]),
        -Units.degreesToRadians(array[4])
      )
    )
  }

  private fun coordinateArraytoTargetCorners(coordinateArray: Array<Double>): List<TargetCorner> {
    return listOf(
      TargetCorner(coordinateArray[0], coordinateArray[1]),
      TargetCorner(coordinateArray[2], coordinateArray[3]),
      TargetCorner(coordinateArray[4], coordinateArray[5]),
      TargetCorner(coordinateArray[6], coordinateArray[7])
    )
  }

  private fun stringToDoubleArray(string: String): Array<Double> {
    return string.substring(1, string.length - 1).split(",").map { it.toDouble() }.toTypedArray()
  }

  private fun ptsToDoubleArray(string: String): Array<Double> {
    return stringToDoubleArray(
      string.substring(1, string.length - 1).replace("[", "").replace("]", "")
    )
  }
}
