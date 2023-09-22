package com.team4099.lib.math

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters

class Zone2d(var vertices: List<Translation2d>) {
  fun containsPose(pose: Pose2d): Boolean {
    var isInside = false
    val minX = vertices.minOf { it.x }
    val minY = vertices.minOf { it.y }
    val maxX = vertices.maxOf { it.x }
    val maxY = vertices.maxOf { it.y }

    if ((pose.x !in minX..maxX) || (pose.y !in minY..maxY)) {
      return false
    }

    var vBIndex = vertices.size - 1
    for (vAIndex in vertices.indices) {
      if ((vertices[vAIndex].y > pose.y) != (vertices[vBIndex].y > pose.y) &&
        pose.x <
        (
          (vertices[vBIndex].x - vertices[vAIndex].x).inMeters *
            (pose.y - vertices[vAIndex].y).inMeters /
            (vertices[vBIndex].y - vertices[vAIndex].y).inMeters +
            vertices[vAIndex].x.inMeters
          )
          .meters
      ) {
        isInside = !isInside
      }
      vBIndex = vAIndex
    }

    return isInside
  }
}
