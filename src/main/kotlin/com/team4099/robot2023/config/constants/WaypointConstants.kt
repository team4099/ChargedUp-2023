package com.team4099.robot2023.config.constants

import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.Waypoint
import edu.wpi.first.math.geometry.Rotation2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians

object WaypointConstants {
  enum class SubstationPoints(val waypoints: List<Waypoint>) {
    CloseLeftCommunity(
      listOf(
        Waypoint(
          Translation2d(3.meters, 4.64.meters).translation2d,
          holonomicRotation = Rotation2d(180.degrees.inRadians)
        )
      )
    ),
    CloseCenterCommunity(
      listOf(
        Waypoint(
          Translation2d(2.01.meters, 3.48.meters).translation2d,
          holonomicRotation = Rotation2d(180.degrees.inRadians)
        )
      )
    ),
    CloseRightCommunity(
      listOf(
        Waypoint(
          Translation2d(2.01.meters, 1.47.meters).translation2d,
          holonomicRotation = Rotation2d(180.degrees.inRadians)
        )
      )
    ),
    CloseLeftLane(listOf(Waypoint(Translation2d(8.12.meters, 5.19.meters).translation2d))),
    CloseLoadingZoneLane(listOf(Waypoint(Translation2d(8.12.meters, 6.75.meters).translation2d))),
    CloseCenterLeftLane(listOf(Waypoint(Translation2d(8.12.meters, 3.8.meters).translation2d))),
    CloseCenterRightLane(listOf(Waypoint(Translation2d(8.12.meters, 3.8.meters).translation2d))),
    CloseRightLane(listOf(Waypoint(Translation2d(6.meters, 0.85.meters).translation2d))),
    FarRightLane(listOf(Waypoint(Translation2d(10.16.meters, 2.meters).translation2d))),
    FarCenterLeftLane(
      listOf(
        Waypoint( // excess
          Translation2d(10.9.meters, 3.81.meters).translation2d
        )
      )
    ),
    FarCenterRightLane(
      listOf(
        Waypoint( // excess
          Translation2d(10.9.meters, 3.81.meters).translation2d
        )
      )
    ),
    FarLeftLane(listOf(Waypoint(Translation2d(10.9.meters, 4.69.meters).translation2d))),
    FarLoadingZoneLane(listOf(Waypoint(Translation2d(12.66.meters, 6.64.meters).translation2d))),
    FarLoadingZone(
      listOf(
        Waypoint(
          Translation2d(14.25.meters, 6.69.meters).translation2d,
          null,
          Rotation2d(90.degrees.inRadians)
        )
      )
    )
  }

  object SubstationPaths {
    var loadingZone = SubstationPoints.FarLoadingZone.waypoints

    var farLoadingZoneLane = SubstationPoints.FarLoadingZoneLane.waypoints + loadingZone
    var farRightLane =
      SubstationPoints.FarRightLane.waypoints +
        SubstationPoints.FarLeftLane.waypoints +
        farLoadingZoneLane
    var excessFarLanes = SubstationPoints.FarLeftLane.waypoints + farLoadingZoneLane

    var closeLoadingZoneLane = SubstationPoints.CloseLoadingZoneLane.waypoints + loadingZone
    var closeLeftLane = SubstationPoints.CloseLeftLane.waypoints + loadingZone
    var closeCenterLeftLane = SubstationPoints.CloseCenterLeftLane.waypoints + farLoadingZoneLane
    var closeCenterRightLane = SubstationPoints.CloseCenterRightLane.waypoints + closeCenterLeftLane
    var closeRightLane = SubstationPoints.CloseRightLane.waypoints + closeCenterLeftLane
    var closeLeftCommunity = SubstationPoints.CloseLeftCommunity.waypoints + closeLeftLane
    var closeCenterCommunity = SubstationPoints.CloseCenterCommunity.waypoints + closeLeftCommunity
    var closeRightCommunity = SubstationPoints.CloseRightCommunity.waypoints + closeCenterCommunity

    fun getPath(zone: Zone2d?): List<Waypoint> {
      return when (zone) {
        FieldConstants.Zones.closeLeftCommunity -> closeLeftCommunity
        FieldConstants.Zones.closeCenterCommunity -> closeCenterCommunity
        FieldConstants.Zones.closeRightCommunity -> closeRightCommunity
        FieldConstants.Zones.closeLeftLane -> closeLeftLane
        FieldConstants.Zones.closeCenterLeftLane -> closeCenterLeftLane
        FieldConstants.Zones.closeCenterRightLane -> closeCenterRightLane
        FieldConstants.Zones.closeRightLane -> closeRightLane
        FieldConstants.Zones.closeLoadingZoneLane -> closeLoadingZoneLane
        FieldConstants.Zones.farLoadingZone -> loadingZone
        FieldConstants.Zones.farLeftLane -> excessFarLanes
        FieldConstants.Zones.farRightLane -> farRightLane
        FieldConstants.Zones.farLoadingZoneLane -> farLoadingZoneLane
        FieldConstants.Zones.farCenterLeftLane -> excessFarLanes
        FieldConstants.Zones.farCenterRightLane -> excessFarLanes
        null -> listOf<Waypoint>()
        else -> listOf<Waypoint>()
      }
    }
  }
}
