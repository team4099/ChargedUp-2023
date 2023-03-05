package com.team4099.robot2023.subsystems.vision.camera

import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.ObjectMapper
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.util.Alert
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.IntegerSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.wpilibj.Timer
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro

class CameraIONorthstar(val identifier: String) : CameraIO {
  private var cameraId = 0
  private var cameraResolutionWidth = 1600
  private var cameraResolutionHeight = 1200
  private var cameraAutoExposure = 1
  private var cameraExposure = 25
  private var cameraGain = 25

  private val observationSubscriber: DoubleArraySubscriber
  private val fpsSubscriber: IntegerSubscriber

  private val disconnectedTimeout = 0.5
  private val disconnectedAlert: Alert
  private val disconnectedTimer = Timer()

  init {
    val northStarTable = NetworkTableInstance.getDefault().getTable(identifier)

    val configTable = northStarTable.getSubTable("config")

    configTable.getIntegerTopic("camera_id").publish().set(cameraId.toLong())
    configTable
      .getIntegerTopic("camera_resolution_width")
      .publish()
      .set(cameraResolutionWidth.toLong())
    configTable
      .getIntegerTopic("camera_resolution_height")
      .publish()
      .set(cameraResolutionHeight.toLong())
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure.toLong())
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure.toLong())
    configTable.getIntegerTopic("camera_gain").publish().set(cameraGain.toLong())
    configTable
      .getDoubleTopic("fiducial_size_m")
      .publish()
      .set(FieldConstants.aprilTagLength.inMeters)
    try {
      configTable
        .getStringTopic("tag_layout")
        .publish()
        .set(ObjectMapper().writeValueAsString(FieldConstants.wpilibFieldLayout))
    } catch (e: JsonProcessingException) {
      throw RuntimeException("Failed to serialize AprilTag layout JSON for Northstar")
    }

    val outputTable = northStarTable.getSubTable("output")
    observationSubscriber =
      outputTable
        .getDoubleArrayTopic("observations")
        .subscribe(
          DoubleArray(0), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)
        )
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0)

    disconnectedAlert = Alert("No data from $identifier", Alert.AlertType.ERROR)
    disconnectedTimer.start()
  }

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    val queue = observationSubscriber.readQueue()
    val timestamps = mutableListOf<Time>()
    val frames = mutableListOf<DoubleArray>()
    for (i in queue.indices) {
      if (queue[i].value.isNotEmpty()) {
        timestamps.add(queue[i].timestamp.micro.seconds)
        frames.add(queue[i].value)
      }
    }

    inputs.timestamps = timestamps
    inputs.frames = frames
    inputs.fps = fpsSubscriber.get().toDouble()

    // Update disconnected alert
    if (queue.isNotEmpty()) {
      disconnectedTimer.reset()
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout))
  }

  private fun printDoubleArray(a: DoubleArray){
    for (b in a){
      println(b)
    }
  }
}
