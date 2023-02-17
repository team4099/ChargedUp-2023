package com.team4099.lib.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import java.util.NavigableMap
import java.util.TreeMap

class PoseEstimator(stateStdDevs: Matrix<N3, N1>) {
  private var basePose = Pose2d()
  /** Returns the latest robot pose based on drive and vision data. */
  var latestPose = Pose2d()
    private set
  private val updates: NavigableMap<Double, PoseUpdate> = TreeMap()
  private val q = Matrix(Nat.N3(), Nat.N1())

  /** Resets the odometry to a known pose. */
  fun resetPose(pose: Pose2d) {
    basePose = pose
    updates.clear()
    update()
  }

  /** Records a new drive movement. */
  fun addDriveData(timestamp: Double, twist: Twist2d) {
    updates[timestamp] = PoseUpdate(twist, mutableListOf<VisionUpdate>())
    update()
  }

  /** Records a new set of vision updates. */
  fun addVisionData(visionData: List<TimestampedVisionUpdate>) {
    for (timestampedVisionUpdate in visionData) {
      val timestamp: Double = timestampedVisionUpdate.timestamp
      val visionUpdate = VisionUpdate(timestampedVisionUpdate.pose, timestampedVisionUpdate.stdDevs)
      if (updates.containsKey(timestamp)) {
        // There was already an update at this timestamp, add to it
        val oldVisionUpdates: MutableList<VisionUpdate>? = updates[timestamp]?.visionUpdates
        oldVisionUpdates?.add(visionUpdate)
        oldVisionUpdates?.sortWith(VisionUpdate.compareDescStdDev)
      } else {
        // Insert a new update
        val prevUpdate = updates.floorEntry(timestamp)
        val nextUpdate = updates.ceilingEntry(timestamp)
        if (prevUpdate == null || nextUpdate == null) {
          // Outside the range of existing data
          return
        }

        // Create partial twists (prev -> vision, vision -> next)
        val twist0: Twist2d =
          multiplyTwist(
            nextUpdate.value.twist,
            (timestamp - prevUpdate.key) / (nextUpdate.key - prevUpdate.key)
          )
        val twist1: Twist2d =
          multiplyTwist(
            nextUpdate.value.twist,
            (nextUpdate.key - timestamp) / (nextUpdate.key - prevUpdate.key)
          )

        // Add new pose updates
        val newVisionUpdates = mutableListOf<VisionUpdate>()
        newVisionUpdates.add(visionUpdate)
        newVisionUpdates.sortWith(VisionUpdate.compareDescStdDev)
        updates[timestamp] = PoseUpdate(twist0, newVisionUpdates)
        updates[nextUpdate.key] = PoseUpdate(twist1, nextUpdate.value.visionUpdates)
      }
    }

    // Recalculate latest pose once
    update()
  }

  /** Clears old data and calculates the latest pose. */
  private fun update() {
    // Clear old data and update base pose
    while (updates.size > 1 &&
      updates.firstKey() < Timer.getFPGATimestamp() - HISTORY_LENGTH.inSeconds
    ) {
      val (_, value) = updates.pollFirstEntry()
      basePose = value.apply(basePose, q)
    }

    // Update latest pose
    latestPose = basePose
    for ((_, value) in updates) {
      latestPose = value.apply(latestPose, q)
    }
  }

  private fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d {
    return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
   * vision updates.
   */
  private class PoseUpdate(var twist: Twist2d, var visionUpdates: MutableList<VisionUpdate>) {
    fun apply(lastPose: Pose2d, q: Matrix<N3, N1>): Pose2d {
      // Apply drive twist
      var pose = lastPose.exp(twist)

      // Apply vision updates
      for (visionUpdate in visionUpdates) {
        // Calculate Kalman gains based on std devs
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        val visionK = Matrix(Nat.N3(), Nat.N3())
        val r = DoubleArray(3)
        for (i in 0..2) {
          r[i] = visionUpdate.stdDevs.get(i, 0) * visionUpdate.stdDevs.get(i, 0)
        }
        for (row in 0..2) {
          if (q[row, 0] == 0.0) {
            visionK[row, row] = 0.0
          } else {
            visionK[row, row] = q[row, 0] / (q[row, 0] + Math.sqrt(q[row, 0] * r[row]))
          }
        }

        // Calculate twist between current and vision pose
        val visionTwist = pose.log(visionUpdate.pose)

        // Multiply by Kalman gain matrix
        val twistMatrix =
          visionK.times(
            VecBuilder.fill(
              visionTwist.dx.inMeters, visionTwist.dy.inMeters, visionTwist.dtheta.inRadians
            )
          )

        // Apply twist
        pose =
          pose.exp(
            Twist2d(
              twistMatrix[0, 0].meters, twistMatrix[1, 0].meters, twistMatrix[2, 0].radians
            )
          )
      }
      return pose
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  data class VisionUpdate(val pose: Pose2d, val stdDevs: Matrix<N3, N1>) {
    companion object {
      val compareDescStdDev =
        java.util.Comparator { a: VisionUpdate, b: VisionUpdate ->
          -(a.stdDevs.get(0, 0) + a.stdDevs.get(1, 0)).compareTo(
            b.stdDevs.get(0, 0) + b.stdDevs.get(1, 0)
          )
        }
    }
  }

  /** Represents a single vision pose with a timestamp and associated standard deviations. */
  data class TimestampedVisionUpdate(
    var timestamp: Double,
    var pose: Pose2d,
    var stdDevs: Matrix<N3, N1>
  )
  companion object {
    private val HISTORY_LENGTH = 0.3.seconds
  }

  init {
    for (i in 0..2) {
      q[i, 0] = stateStdDevs[i, 0] * stateStdDevs[i, 0]
    }
  }
}
