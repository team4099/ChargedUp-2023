package com.team4099.lib.pathfollow

import com.team4099.robot2023.util.Alert
import edu.wpi.first.math.spline.PoseWithCurvature
import edu.wpi.first.math.spline.Spline
import edu.wpi.first.math.spline.SplineHelper
import edu.wpi.first.math.spline.SplineParameterizer
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.atan2

/**
 * A path on the XY plane constructed with cubic splines.
 *
 * Heading of holonomic drivetrains can be controlled at any waypoint.
 */
class Path(
  val startingPose: Pose2d,
  val endingPose: Pose2d,
  val startingVelocity: Velocity2d,
  val endingVelocity: Velocity2d,
  val useCubic: Boolean = false
) {
  val headingPointMap = sortedMapOf<Int, Angle>()
  var splinePoints = mutableListOf<PoseWithCurvature>()
  private val headingSplineMap = mutableMapOf<Int, Angle>()
  private val waypoints = mutableListOf<Translation2d>()
  var built = false
  val addTranslationAlert: Alert =
    Alert("Failed to add translation to built path", Alert.AlertType.ERROR)
  val alreadyBuiltAlert: Alert = Alert("Failed build already built path", Alert.AlertType.ERROR)

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
  fun addWaypoint(nextTranslation: Translation2d, heading: Angle? = null) {
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

    val splines: Array<out Spline>

    if (useCubic) {

      // Make the starting curvature directly towards the first point
      val startHeading =
        if (startingVelocity.magnitude < velocityThreshold)
          atan2(
            ((waypoints.firstOrNull() ?: endingPose.translation).y - startingPose.y)
              .inMeters,
            ((waypoints.firstOrNull() ?: endingPose.translation).x - startingPose.x)
              .inMeters
          )
            .radians
        else startingVelocity.heading

      val endHeading =
        if (endingVelocity.magnitude < velocityThreshold)
          (
            (
              atan2(
                ((waypoints.lastOrNull() ?: startingPose.translation).y - endingPose.y)
                  .inMeters,
                ((waypoints.lastOrNull() ?: startingPose.translation).x - endingPose.x)
                  .inMeters
              ) + PI
              ) % (2 * PI)
            )
            .radians
        else endingVelocity.heading

      // Cubic spline generation
      val controlVectors =
        SplineHelper.getCubicControlVectorsFromWaypoints(
          startingPose.copy(rotation = startHeading).pose2d,
          waypointTranslation2ds,
          endingPose.copy(rotation = endHeading).pose2d
        )

      // Create a list of splines
      splines =
        SplineHelper.getCubicSplinesFromControlVectors(
          controlVectors.first(), waypointTranslation2ds, controlVectors.last()
        )
    } else {

      // Quintic spline generation
      val waypointsWithHeadings =
        mutableListOf<Pose2dWPILIB>(
          Pose2dWPILIB(
            startingPose.translation.translation2d, startingPose.rotation.inRotation2ds
          )
        )
      for (waypointIndex in 0..waypoints.size) {
        if (headingSplineMap[waypointIndex] != null) {
          waypointsWithHeadings.add(
            Pose2dWPILIB(
              waypoints[waypointIndex].translation2d,
              headingSplineMap[waypointIndex]!!.inRotation2ds
            )
          )
        }
      }

      val firstWaypoint = waypointsWithHeadings.firstOrNull() ?: endingPose.pose2d

      // Make the starting curvature directly towards the first point
      val quinticStartHeading =
        if (startingVelocity.magnitude < velocityThreshold)
          atan2(
            firstWaypoint.y - startingPose.y.inMeters,
            firstWaypoint.x - startingPose.x.inMeters
          )
            .radians
        else startingVelocity.heading

      waypointsWithHeadings[0] =
        Pose2dWPILIB(startingPose.translation.translation2d, quinticStartHeading.inRotation2ds)

      val lastWayPoint = waypointsWithHeadings.lastOrNull() ?: startingPose.pose2d

      val quinticEndHeading =
        if (endingVelocity.magnitude < velocityThreshold)
          (
            (
              atan2(
                (lastWayPoint.y - endingPose.y.inMeters),
                (lastWayPoint.x - endingPose.x.inMeters)
              ) + PI
              ) % (2 * PI)
            )
            .radians
        else endingVelocity.heading

      waypointsWithHeadings.add(
        Pose2dWPILIB(endingPose.translation.translation2d, quinticEndHeading.inRotation2ds)
      )

      for (waypoint in waypointsWithHeadings) {
        println(waypoint)
      }

      // Creates a list of splines
      splines = SplineHelper.getQuinticSplinesFromWaypoints(waypointsWithHeadings)
    }

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

  companion object {
    val velocityThreshold = 0.2.meters.perSecond
  }
}
