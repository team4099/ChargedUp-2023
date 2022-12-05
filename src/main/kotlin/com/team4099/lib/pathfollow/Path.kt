package com.team4099.lib.pathfollow

import com.team4099.lib.geometry.Pose
import com.team4099.lib.geometry.Translation
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.radians
import com.team4099.robot2022.util.Alert
import edu.wpi.first.math.spline.PoseWithCurvature
import edu.wpi.first.math.spline.SplineHelper
import edu.wpi.first.math.spline.SplineParameterizer
import kotlin.math.PI
import kotlin.math.atan2

/**
 * A path on the XY plane constructed with cubic splines.
 *
 * Heading of holonomic drivetrains can be controlled at any waypoint.
 */
class Path constructor(val startingPose: Pose, val endingPose: Pose) {
  val headingPointMap = sortedMapOf<Int, Angle>()
  var splinePoints = mutableListOf<PoseWithCurvature>()
  private val headingSplineMap = mutableMapOf<Int, Angle>()
  private val waypoints = mutableListOf<Translation>()
  var built = false
  val addTranslationAlert: Alert =
    Alert("Failed to add translation to built path", Alert.AlertType.ERROR)
  val alreadyBuiltAlert: Alert =
    Alert("Failed com.team4099.lib.annotations.build already built path", Alert.AlertType.ERROR)

  /**
   * Add a waypoint to the middle of this path.
   *
   * The robot will pass through [nextTranslation] as it travels along the path. If [heading] is set
   * the robot will have that heading at this location, otherwise heading will be interpolated
   * between the previous waypoint with heading specified and the next waypoint with heading
   * specified, including the start and end poses.
   * @param nextTranslation The location of the waypoint.
   * @param heading The target heading at this waypoint, null if the heading at this waypoint does
   * not matter.
   */
  fun addWaypoint(nextTranslation: Translation, heading: Angle? = null) {
    if (built) {
      addTranslationAlert.set(true)
      return
    }

    if (heading != null) {
      headingSplineMap[waypoints.size] = heading
    }
    waypoints.add(nextTranslation)
  }

  /** Build the path after all desired waypoints have been added. */
  fun build() {
    if (built) {
      alreadyBuiltAlert.set(true)
      return
    }

    // Create control vectors from the start and end waypoint
    val waypointTranslation2ds = waypoints.map { it.translation2d }.toTypedArray()

    // Make the starting curvature directly towards the first point
    val startHeading =
      atan2(
        ((waypoints.firstOrNull() ?: endingPose.translation).y - startingPose.y).inMeters,
        ((waypoints.firstOrNull() ?: endingPose.translation).x - startingPose.x).inMeters
      )
        .radians

    val endHeading =
      (
        (
          atan2(
            ((waypoints.lastOrNull() ?: startingPose.translation).y - endingPose.y).inMeters,
            ((waypoints.lastOrNull() ?: startingPose.translation).x - endingPose.x).inMeters
          ) +
            PI
          ) % (2 * PI)
        )
        .radians

    val controlVectors =
      SplineHelper.getCubicControlVectorsFromWaypoints(
        startingPose.copy(theta = startHeading).pose2d,
        waypointTranslation2ds,
        endingPose.copy(theta = endHeading).pose2d
      )

    // Create a list of splines
    val splines =
      listOf(
        *SplineHelper.getCubicSplinesFromControlVectors(
          controlVectors.first(), waypointTranslation2ds, controlVectors.last()
        )
      )

    // Create the vector of spline points.
    splinePoints = mutableListOf()

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0))

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    splines.forEachIndexed { index, spline ->
      val points = SplineParameterizer.parameterize(spline)

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size))

      // Map spline index for heading to point index
      val splineHeading = headingSplineMap[index]
      if (splineHeading != null) {
        headingPointMap[splinePoints.size - 1] = splineHeading
      }
    }
    built = true
  }
}
