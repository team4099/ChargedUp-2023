package com.team4099.lib.pathfollow

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.derived.Angle

data class PoseWithHeading(val pose: Pose2d, val heading: Angle)
