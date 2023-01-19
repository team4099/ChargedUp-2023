package com.team4099.robot2023.subsystems.vision

import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import edu.wpi.first.networktables.NetworkTableInstance
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import kotlin.collections.HashMap

object VisionIOLimelight : VisionIO {
  val jsonType = object : TypeToken<Map<String, Map<String, Any>>>() {}.type

  val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")
  val limelightJsonSub = limelightTable.getStringTopic("json").getEntry("[]")
  val latencySub =
    limelightTable
      .getDoubleTopic("tl")
      .getEntry(0.0) // add at least 11 ms for image capture latency

  override fun updateInputs(inputs: VisionIO.VisionInputs) {
    val limelightJson =
      Gson().fromJson<Map<String, Map<String, Any>>>(limelightJsonSub.get(), jsonType)
    val results = limelightJson["Results"]
    val fiducials: List<Map<String, Any>> =
      (results?.get("Fiducial") ?: listOf<Map<String, Any>>()) as List<HashMap<String, Any>>

    val trackedTargets =
      fiducials.map {
        PhotonTrackedTarget(
          it["tx"].toString().toDouble(),
          it["ty"].toString().toDouble(),
          it["ta"].toString().toDouble(),
          0.0, // unused
          it["fID"].toString().toDouble().toInt(),
          targetSpaceArrayToTransform(stringToDoubleArray(it["t6t_rs"].toString())).transform3d,
          targetSpaceArrayToTransform(stringToDoubleArray(it["t6t_rs"].toString())).transform3d,
          0.0,
          coordinateArraytoTargetCorners(ptsToDoubleArray(it["pts"].toString()))
        )
      }
    inputs.photonResults = listOf(PhotonPipelineResult(latencySub.get() + 11, trackedTargets))
  }

  fun targetSpaceArrayToTransform(array: Array<Double>): Transform3d {
    // https://docs.limelightvision.io/en/latest/coordinate_systems_fiducials.html#target-space
    return Transform3d(
      Translation3d(array[2].meters, array[0].meters, -array[1].meters),
      Rotation3d(array[3].degrees, array[4].degrees, array[5].degrees)
    )
  }

  fun coordinateArraytoTargetCorners(coordinateArray: Array<Double>): List<TargetCorner> {
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
