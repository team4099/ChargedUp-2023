package com.team4099.robot2023.util

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import java.util.NavigableMap
import java.util.TreeMap

class PoseEstimator(stateStdDevs: Matrix<N3?, N1?>) {
  private var basePose: Pose2d = Pose2d()
  private var latestPose: Pose2d = Pose2d()
  private val updates: NavigableMap<Double, PoseUpdate> = TreeMap()
  private val q: Matrix<N3?, N1?> = Matrix(Nat.N3(), Nat.N1())

  /** Returns the latest robot pose based on drive and vision data. */
  fun getLatestPose(): Pose2d {
    return latestPose
  }

  /** Resets the odometry to a known pose. */
  fun resetPose(pose: Pose2d) {
    basePose = pose
    updates.clear()
    update()
  }

  /** Records a new drive movement. */
  fun addDriveData(timestamp: Double, twist: Twist2d) {
    updates[timestamp] = PoseUpdate(twist, ArrayList<VisionUpdate>())
    update()
  }

  /** Records a new set of vision updates. */
  fun addVisionData(visionData: List<TimestampedVisionUpdate>) {
    for (timestampedVisionUpdate in visionData) {
      val timestamp: Double = timestampedVisionUpdate.timestamp.inSeconds
      val visionUpdate =
        VisionUpdate(
          timestampedVisionUpdate.pose,
          timestampedVisionUpdate.stdDevs,
          timestampedVisionUpdate.fromVision
        )
      if (updates.containsKey(timestamp)) {
        // There was already an update at this timestamp, add to it
        val oldVisionUpdates: ArrayList<VisionUpdate> = updates[timestamp]!!.visionUpdates
        oldVisionUpdates.add(visionUpdate)
        oldVisionUpdates.sortWith(VisionUpdate.compareDescStdDev)
      } else {
        // Insert a new update
        val prevUpdate = updates.floorEntry(timestamp)
        val nextUpdate = updates.ceilingEntry(timestamp)
        if (prevUpdate == null || nextUpdate == null) {
          // Outside the range of existing data
          return
        }

        // Create partial twists (prev -> vision, vision -> next)
        val twist0 =
          multiplyTwist(
            nextUpdate.value.twist2d,
            (timestamp - prevUpdate.key) / (nextUpdate.key - prevUpdate.key)
          )
        val twist1 =
          multiplyTwist(
            nextUpdate.value.twist2d,
            (nextUpdate.key - timestamp) / (nextUpdate.key - prevUpdate.key)
          )

        // Add new pose updates
        val newVisionUpdates = ArrayList<VisionUpdate>()
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
    while (updates.size > 1 && updates.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
      val (_, value) = updates.pollFirstEntry()
      basePose = value.apply(basePose, q)
    }

    // Update latest pose
    latestPose = basePose
    for (updateEntry in updates.entries) {
      latestPose = updateEntry.value.apply(latestPose, q)
    }

    for (update in updates) {
      if (update.value.visionUpdates.size > 0 && update.value.visionUpdates[0].fromVision) {
        Logger.getInstance().recordOutput("Vision/Buffer/Vision", update.key)

        Logger.getInstance()
          .recordOutput("Vision/Buffer/VisionPose", update.value.visionUpdates[0].pose.pose2d)
      } else {
        Logger.getInstance().recordOutput("Vision/Buffer/Drivetrain", update.key)
      }
    }
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
   * vision updates.
   */
  private class PoseUpdate(val twist2d: Twist2d, val visionUpdates: ArrayList<VisionUpdate>) {
    fun apply(lastPose: Pose2d, q: Matrix<N3?, N1?>): Pose2d {
      // Apply drive twist
      var pose = lastPose.exp(twist2d)

      // Apply vision updates
      for (visionUpdate in visionUpdates) {
        // Calculate Kalman gains based on std devs
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        val visionK: Matrix<N3, N3> = Matrix(Nat.N3(), Nat.N3())
        val r = DoubleArray(3)
        for (i in 0..2) {
          r[i] = visionUpdate.stdDevs.get(i, 0) * visionUpdate.stdDevs.get(i, 0)
        }
        for (row in 0..2) {
          if (q.get(row, 0) === 0.0) {
            visionK.set(row, row, 0.0)
          } else {
            visionK.set(
              row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row]))
            )
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
              twistMatrix.get(0, 0).meters,
              twistMatrix.get(1, 0).meters,
              twistMatrix.get(2, 0).radians
            )
          )
      }
      return pose
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  class VisionUpdate(
    val pose: Pose2d,
    val stdDevs: Matrix<N3, N1>,
    val fromVision: Boolean = false
  ) {
    companion object {
      val compareDescStdDev = Comparator { a: VisionUpdate, b: VisionUpdate ->
        -(a.stdDevs.get(0, 0) + a.stdDevs.get(1, 0)).compareTo(
          b.stdDevs.get(0, 0) + b.stdDevs.get(1, 0)
        )
      }
    }
  }

  /** Represents a single vision pose with a timestamp and associated standard deviations. */
  class TimestampedVisionUpdate(
    val timestamp: Time,
    val pose: Pose2d,
    val stdDevs: Matrix<N3, N1>,
    val fromVision: Boolean = false
  )
  companion object {
    private const val historyLengthSecs = 0.3
  }

  init {
    for (i in 0..2) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0))
    }
  }
}
