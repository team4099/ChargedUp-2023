package com.team4099.robot2023.config.constants

import com.team4099.lib.math.Zone2d
import com.team4099.lib.trajectory.Waypoint
import edu.wpi.first.math.geometry.Rotation2d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians

object WaypointConstants {
  enum class SubstationPoints(val waypoint : Waypoint) {
    CloseCommunity(Waypoint(Translation2d(2.3.meters, 4.69.meters).translation2d)),
    CloseLeftLane(Waypoint(Translation2d(8.12.meters, 5.19.meters).translation2d)),
    CloseLoadingZoneLane(Waypoint(Translation2d(8.12.meters, 6.75.meters).translation2d)),
    CloseCenterLeftLane(Waypoint(Translation2d(8.12.meters, 3.8.meters).translation2d)),
    CloseCenterRightLane(Waypoint(Translation2d(8.12.meters, 3.8.meters).translation2d)),
    CloseRightLane(Waypoint(Translation2d(8.meters, 1.45.meters).translation2d)),
    FarRightLane(Waypoint(Translation2d(10.75.meters, 1.45.meters).translation2d)),
    FarCenterLeftLane(
      Waypoint( // excess
        Translation2d(10.9.meters, 3.81.meters).translation2d
      )
    ),
    FarCenterRightLane(
      Waypoint( // excess
        Translation2d(10.9.meters, 3.81.meters).translation2d
      )
    ),
    FarLeftLane(Waypoint(Translation2d(12.2.meters, 5.2.meters).translation2d)),
    FarLoadingZoneLane(Waypoint(Translation2d(12.66.meters, 6.64.meters).translation2d)),
    FarLoadingZone(
      Waypoint(
        Translation2d(14.25.meters, 6.69.meters).translation2d,
        null,
        Rotation2d(90.degrees.inRadians)
      )
    )
  }

  object SubstationPaths {
    var loadingZone = listOf<Waypoint>(SubstationPoints.FarLoadingZone.waypoint)

    var farLoadingZoneLane =
      listOf<Waypoint>(SubstationPoints.FarLoadingZoneLane.waypoint) + loadingZone
    var farRightLane = listOf<Waypoint>(SubstationPoints.FarRightLane.waypoint) + farLoadingZoneLane
    var excessFarLanes = listOf<Waypoint>(SubstationPoints.FarLeftLane.waypoint) + farLoadingZoneLane

    var closeLoadingZoneLane =
      listOf<Waypoint>(SubstationPoints.CloseLoadingZoneLane.waypoint) + loadingZone
    var closeLeftLane =
      listOf<Waypoint>(SubstationPoints.CloseLeftLane.waypoint) + loadingZone
    var closeCenterLeftLane =
      listOf<Waypoint>(SubstationPoints.CloseCenterLeftLane.waypoint) + farLoadingZoneLane
    var closeCenterRightLane =
      listOf<Waypoint>(SubstationPoints.CloseCenterRightLane.waypoint) + closeCenterLeftLane
    var closeRightLane =
      listOf<Waypoint>(SubstationPoints.CloseRightLane.waypoint) + closeCenterLeftLane

    var closeCommunity = listOf<Waypoint>(SubstationPoints.CloseCommunity.waypoint) + closeLeftLane

    fun getPath(zone: Zone2d?): List<Waypoint> {
      return when (zone) {
        FieldConstants.Zones.closeCommunity -> closeCommunity
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
