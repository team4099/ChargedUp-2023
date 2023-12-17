package com.team4099.lib.trajectory

import edu.wpi.first.math.geometry.Rotation2d
import java.util.TreeMap

/**
 * Represents a sequence of timed rotations. The position and velocity of the robot is calculated to
 * minimize acceleration.
 */
class RotationSequence(sequence: TreeMap<Double, Rotation2d>?) {
  private val sequence = TreeMap<Double, Rotation2d>()

  /** Constructs a rotation sequence from a series of timed rotation positions. */
  init {
    this.sequence.putAll(sequence!!)
  }

  /**
   * Sample the rotation sequence at a point in time.
   *
   * @param timeSeconds The point in time since the beginning of the rotation sequence to sample.
   * @return The state at that point in time.
   */
  fun sample(timeSeconds: Double): State {
    var positionRadians: Double
    val velocityRadiansPerSec: Double
    val lastPoint = sequence.floorEntry(timeSeconds)
    val nextPoint = sequence.higherEntry(timeSeconds)

    if (lastPoint == null && nextPoint == null) { // No points in sequence
      positionRadians = 0.0
      velocityRadiansPerSec = 0.0
    } else if (lastPoint == null) { // Before start of sequence
      positionRadians = nextPoint!!.value.radians
      velocityRadiansPerSec = 0.0
    } else if (nextPoint == null) { // Before end of sequence
      positionRadians = lastPoint.value.radians
      velocityRadiansPerSec = 0.0
    } else {
      val accelerationRadiansPerSec2 =
        (
          4 * nextPoint.value.minus(lastPoint.value).radians /
            Math.pow(nextPoint.key - lastPoint.key, 2.0)
          )
      if (timeSeconds < (nextPoint.key + lastPoint.key) / 2) { // Accelerating
        positionRadians =
          (
            lastPoint.value.radians +
              (accelerationRadiansPerSec2 / 2 * Math.pow(timeSeconds - lastPoint.key, 2.0))
            )
        velocityRadiansPerSec = (timeSeconds - lastPoint.key) * accelerationRadiansPerSec2
      } else { // Decelerating
        positionRadians =
          (
            nextPoint.value.radians -
              (accelerationRadiansPerSec2 / 2 * Math.pow(timeSeconds - nextPoint.key, 2.0))
            )
        velocityRadiansPerSec = (nextPoint.key - timeSeconds) * accelerationRadiansPerSec2
      }
    }

    // Keep position within acceptable range
    while (positionRadians > Math.PI) {
      positionRadians -= Math.PI * 2
    }
    while (positionRadians < -Math.PI) {
      positionRadians += Math.PI * 2
    }
    return State(Rotation2d(positionRadians), velocityRadiansPerSec)
  }

  /** Represents a state in a rotation sequence with a position and velocity. */
  class State {
    var position: Rotation2d
    var velocityRadiansPerSec: Double

    constructor() {
      position = Rotation2d()
      velocityRadiansPerSec = 0.0
    }

    /**
     * Constructs a State with the specified parameters.
     *
     * @param position The position at this point in the rotation sequence.
     * @param velocityRadiansPerSec The velocity at this point in the rotation sequence.
     */
    constructor(position: Rotation2d, velocityRadiansPerSec: Double) {
      this.position = position
      this.velocityRadiansPerSec = velocityRadiansPerSec
    }
  }
}
