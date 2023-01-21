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
import org.team4099.lib.units.derived.radians

object VisionIOLimelight : VisionIO {
  private val jsonType = object : TypeToken<HashMap<String, HashMap<String, Any>>>() {}.type

  val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")
  val limelightJson =
    Gson()
      .fromJson<HashMap<String, HashMap<String, Any>>>(
        limelightTable.getStringTopic("json").getEntry("[]").toString(), jsonType
      )
  val hasTargetSub = limelightTable.getDoubleTopic("tv").getEntry(0.0)
  val yawSub = limelightTable.getDoubleTopic("tx").getEntry(0.0)
  val pitchSub = limelightTable.getDoubleTopic("ty").getEntry(0.0)
  val latencySub =
    limelightTable
      .getDoubleTopic("tl")
      .getEntry(0.0) // add at least 11 ms for image capture latency
  val areaSub = limelightTable.getDoubleTopic("ta").getEntry(0.0)
  val skewSub = limelightTable.getDoubleTopic("ts").getEntry(0.0)
  val idSub = limelightTable.getDoubleTopic("tid").getEntry(-1.0)
  val camtranSub =
    limelightTable
      .getDoubleArrayTopic("botpose")
      .getEntry(doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
  val cornerCoordinates =
    limelightTable
      .getDoubleArrayTopic("tcornxy")
      .getEntry(doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

  override fun updateInputs(inputs: VisionIO.VisionInputs) {
    val fiducials = limelightJson["Results"]?.get("Fiducial") as List<HashMap<String, Any>>
    val trackedTargets =
      fiducials.map {
        PhotonTrackedTarget(
          it["tx"].toString().toDouble(),
          it["ty"].toString().toDouble(),
          it["ta"].toString().toDouble(),
          limelightJson["Results"]?.get("ts").toString().toDouble(),
          it["fID"].toString().toInt(),
          transformArrayToTransform(camtranSub.get()).transform3d,
          transformArrayToTransform(camtranSub.get()).transform3d,
          0.0,
          coordinateArraytoTargetCorners(cornerCoordinates.get())
        )
      }
    inputs.photonResults = listOf(PhotonPipelineResult(latencySub.get() + 11, trackedTargets))
  }

  fun transformArrayToTransform(array: DoubleArray): Transform3d {
    return Transform3d(
      Translation3d(array[0].meters, array[1].meters, array[2].meters),
      Rotation3d(array[3].radians, array[4].radians, array[5].radians)
    )
  }

  fun coordinateArraytoTargetCorners(coordinateArray: DoubleArray): List<TargetCorner> {
    return listOf(
      TargetCorner(coordinateArray[0], coordinateArray[1]),
      TargetCorner(coordinateArray[2], coordinateArray[3]),
      TargetCorner(coordinateArray[4], coordinateArray[5]),
      TargetCorner(coordinateArray[6], coordinateArray[7])
    )
  }
}
