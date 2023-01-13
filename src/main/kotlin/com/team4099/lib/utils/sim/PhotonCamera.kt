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

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.common.hardware.VisionLEDMode
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.photonvision.PhotonVersion
import org.photonvision.common.dataflow.structures.Packet

/** Represents a camera that is connected to PhotonVision.  */
class PhotonCamera(instance: NetworkTableInstance, val name: String?) {
  val rootTable: NetworkTable
  val rawBytesEntry: NetworkTableEntry
  val driverModeEntry: NetworkTableEntry
  val inputSaveImgEntry: NetworkTableEntry
  val outputSaveImgEntry: NetworkTableEntry
  val pipelineIndexEntry: NetworkTableEntry
  val ledModeEntry: NetworkTableEntry
  val versionEntry: NetworkTableEntry
  private val path: String
  private var lastVersionCheckTime = 0.0
  var packet = Packet(1)

  /**
   * Constructs a PhotonCamera from the name of the camera.
   *
   * @param cameraName The nickname of the camera (found in the PhotonVision UI).
   */
  constructor(cameraName: String?) : this(NetworkTableInstance.getDefault(), cameraName) {}// Clear the packet.

  // Create latest result.

  // Populate packet and create result.

  // Set the timestamp of the result.
  // getLatestChange returns in microseconds so we divide by 1e6 to convert to seconds.

  // Return result.
  /**
   * Returns the latest pipeline result.
   *
   * @return The latest pipeline result.
   */
  val latestResult: PhotonPipelineResult
    get() {
      verifyVersion()

      // Clear the packet.
      packet.clear()

      // Create latest result.
      val ret = PhotonPipelineResult()

      // Populate packet and create result.
      packet.data = rawBytesEntry.getRaw(byteArrayOf())
      if (packet.size < 1) return ret
      ret.createFromPacket(packet)

      // Set the timestamp of the result.
      // getLatestChange returns in microseconds so we divide by 1e6 to convert to seconds.
      ret.timestampSeconds = rawBytesEntry.lastChange / 1e6 - ret.latencyMillis / 1e3

      // Return result.
      return ret
    }
  /**
   * Returns whether the camera is in driver mode.
   *
   * @return Whether the camera is in driver mode.
   */
  /**
   * Toggles driver mode.
   *
   * @param driverMode Whether to set driver mode.
   */
  var driverMode: Boolean
    get() = driverModeEntry.getBoolean(false)
    set(driverMode) {
      driverModeEntry.setBoolean(driverMode)
    }

  /**
   * Request the camera to save a new image file from the input camera stream with overlays. Images
   * take up space in the filesystem of the PhotonCamera. Calling it frequently will fill up disk
   * space and eventually cause the system to stop working. Clear out images in
   * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
   */
  fun takeInputSnapshot() {
    inputSaveImgEntry.setBoolean(true)
  }

  /**
   * Request the camera to save a new image file from the output stream with overlays. Images take
   * up space in the filesystem of the PhotonCamera. Calling it frequently will fill up disk space
   * and eventually cause the system to stop working. Clear out images in
   * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
   */
  fun takeOutputSnapshot() {
    outputSaveImgEntry.setBoolean(true)
  }
  /**
   * Returns the active pipeline index.
   *
   * @return The active pipeline index.
   */
  /**
   * Allows the user to select the active pipeline index.
   *
   * @param index The active pipeline index.
   */
  var pipelineIndex: Int
    get() = pipelineIndexEntry.getNumber(0).toInt()
    set(index) {
      pipelineIndexEntry.setNumber(index)
    }

  /**
   * Returns the current LED mode.
   *
   * @return The current LED mode.
   */
  val lEDMode: VisionLEDMode
    get() {
      val value = ledModeEntry.getNumber(-1).toInt()
      return when (value) {
        0 -> VisionLEDMode.kOff
        1 -> VisionLEDMode.kOn
        2 -> VisionLEDMode.kBlink
        -1 -> VisionLEDMode.kDefault
        else -> VisionLEDMode.kDefault
      }
    }

  /**
   * Sets the LED mode.
   *
   * @param led The mode to set to.
   */
  fun setLED(led: VisionLEDMode) {
    ledModeEntry.setNumber(led.value)
  }

  /**
   * Returns whether the latest target result has targets.
   *
   *
   * This method is deprecated; [PhotonPipelineResult.hasTargets] should be used instead.
   *
   * @return Whether the latest target result has targets.
   */
  @Deprecated(
    """This method should be replaced with {@link PhotonPipelineResult#hasTargets()}
      """
  )
  fun hasTargets(): Boolean {
    return latestResult.hasTargets()
  }

  private fun verifyVersion() {
    if (!VERSION_CHECK_ENABLED) return
    if (Timer.getFPGATimestamp() - lastVersionCheckTime < VERSION_CHECK_INTERVAL) return
    lastVersionCheckTime = Timer.getFPGATimestamp()
    val versionString = versionEntry.getString("")
    if (versionString == "") {
      DriverStation.reportError(
        "PhotonVision coprocessor at path $path not found on NetworkTables!", true
      )
    } else if (!PhotonVersion.versionMatches(versionString)) {
      DriverStation.reportError(
        "Photon version "
          + PhotonVersion.versionString
          + " does not match coprocessor version "
          + versionString
          + "!",
        true
      )
    }
  }

  companion object {
    private var VERSION_CHECK_ENABLED = true
    private const val VERSION_CHECK_INTERVAL: Long = 1
    fun setVersionCheckEnabled(enabled: Boolean) {
      VERSION_CHECK_ENABLED = enabled
    }
  }

  /**
   * Constructs a PhotonCamera from a root table.
   *
   * @param instance The NetworkTableInstance to pull data from. This can be a custom instance in
   * simulation, but should *usually* be the default NTInstance from
   * NetworkTableInstance::getDefault
   * @param cameraName The name of the camera, as seen in the UI.
   */
  init {
    val mainTable = instance.getTable("photonvision")
    rootTable = mainTable.getSubTable(name)
    path = rootTable.path
    rawBytesEntry = rootTable.getEntry("rawBytes")
    driverModeEntry = rootTable.getEntry("driverMode")
    inputSaveImgEntry = rootTable.getEntry("inputSaveImgCmd")
    outputSaveImgEntry = rootTable.getEntry("outputSaveImgCmd")
    pipelineIndexEntry = rootTable.getEntry("pipelineIndex")
    ledModeEntry = mainTable.getEntry("ledMode")
    versionEntry = mainTable.getEntry("version")
  }
}
