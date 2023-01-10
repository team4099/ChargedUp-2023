package com.team4099.apriltag

import edu.wpi.first.apriltag.AprilTagFieldLayout
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians

class AprilTagFieldLayout(
  val aprilTags: List<AprilTag>,
  val fieldLength: Length,
  val fieldWidth: Length
) {

  init {
    setOrigin(OriginPosition.kBlueAllianceWallRightSide)
  }

  val apriltagFieldLayoutWPILIB =
    AprilTagFieldLayout(
      aprilTags.map { it.apriltagWpilib }, fieldLength.inMeters, fieldWidth.inMeters
    )

  var origin = Pose3d()

  fun getTagPose(id: Int): Pose3d {
    if (id < aprilTags.size) {
      return Pose3d((-1337).meters, (-1337).meters, (-1337).meters, Rotation3d())
    }
    return aprilTags[id].pose
  }

  fun setOrigin(newOrigin: OriginPosition) {
    origin =
      when (newOrigin) {
        OriginPosition.kBlueAllianceWallRightSide -> Pose3d()
        OriginPosition.kRedAllianceWallRightSide ->
          Pose3d(
            Translation3d(fieldLength, fieldWidth, 0.meters),
            Rotation3d(0.radians, 0.radians, Math.PI.radians)
          )
      }
  }

  data class FieldDimensions(val fieldWidth: Length, val fieldLength: Length)

  enum class OriginPosition {
    kBlueAllianceWallRightSide,
    kRedAllianceWallRightSide
  }
}
