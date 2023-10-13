package com.team4099.robot2023.config.constants

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.apriltag.AprilTagFieldLayout
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall. Use the [.allianceFlip] and [.allianceFlip] methods to flip these values based on the
 * current alliance color.
 */
object FieldConstants {
  val fieldLength = 651.25.inches
  val fieldWidth = 315.5.inches
  val tapeWidth = 2.0.inches

  val aprilTagLength = 6.0.inches

  val chargeStationEngageAngle = 5.5.degrees

  val homeAprilTags: List<AprilTag> =
    listOf(
      AprilTag(
        1,
        Pose3d(
          40.inches,
          (104.125).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
        )
      ),
      AprilTag(
        2,
        Pose3d(
          (43.125).inches,
          (40.875).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
        )
      ),
      AprilTag(
        3,
        Pose3d(
          (42.875).inches,
          (113.25).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
        )
      ),
      AprilTag(
        4,
        Pose3d(
          (43.125).inches,
          (173.375).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
        )
      ),
      AprilTag(
        5,
        Pose3d(
          (409.5).inches,
          (85.5).inches,
          (27.833).inches,
          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
        )
      ),
      AprilTag(
        6,
        Pose3d(
          (40.45).inches,
          (174.19).inches, // FIRST's diagram has a typo (it says 147.19)
          (18.22).inches,
          Rotation3d()
        )
      ),
      AprilTag(7, Pose3d((40.45).inches, (108.19).inches, (18.22).inches, Rotation3d())),
      AprilTag(8, Pose3d((40.45).inches, (42.19).inches, (18.22).inches, Rotation3d()))
    )

  //  val homeAprilTags: List<AprilTag> =
  //    listOf(
  //      AprilTag(
  //        1,
  //        Pose3d(
  //          40.inches,
  //          (104.125).inches,
  //          (18.22).inches,
  //          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
  //        )
  //      ),
  //      AprilTag(
  //        2,
  //        Pose3d(
  //          0.inches,
  //          (42.125).inches,
  //          (18.22).inches,
  //          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
  //        )
  //      ),
  //      AprilTag(
  //        3,
  //        Pose3d(
  //          0.inches,
  //          (104.125).inches,
  //          (18.22).inches,
  //          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
  //        )
  //      ),
  //      AprilTag(
  //        4,
  //        Pose3d(
  //          (8.13).inches,
  //          (174.19).inches,
  //          (18.22).inches,
  //          Rotation3d(0.0.radians, 0.0.radians, 0.0.radians)
  //        )
  //      ),
  //      AprilTag(
  //        5,
  //        Pose3d(
  //          (409.5).inches,
  //          (85.5).inches,
  //          (27.833).inches,
  //          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
  //        )
  //      ),
  //      AprilTag(
  //        6,
  //        Pose3d(
  //          (40.45).inches,
  //          (174.19).inches, // FIRST's diagram has a typo (it says 147.19)
  //          (18.22).inches,
  //          Rotation3d()
  //        )
  //      ),
  //      AprilTag(7, Pose3d((40.45).inches, (108.19).inches, (18.22).inches, Rotation3d())),
  //      AprilTag(8, Pose3d((280.125).inches, (104.5).inches, (18.22).inches,
  // Rotation3d(0.0.degrees, 0.0.degrees, 180.degrees)))
  //    )

  // AprilTag locations (do not flip for red alliance)
  val aprilTags: List<AprilTag> =
    listOf(
      AprilTag(
        1,
        Pose3d(
          (610.77).inches,
          (42.19).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
        )
      ),
      AprilTag(
        2,
        Pose3d(
          (610.77).inches,
          (108.19).inches,
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
        )
      ),
      AprilTag(
        3,
        Pose3d(
          (610.77).inches,
          (174.19).inches, // FIRST's diagram has a typo (it says 147.19)
          (18.22).inches,
          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
        )
      ),
      AprilTag(
        4,
        Pose3d(
          (636.96).inches,
          (265.74).inches,
          (27.38).inches,
          Rotation3d(0.0.radians, 0.0.radians, Math.PI.radians)
        )
      ),
      AprilTag(
        5,
        Pose3d((14.25).inches, (265.74).inches, (27.38).inches, Rotation3d()),
      ),
      AprilTag(
        6,
        Pose3d(
          (40.45).inches,
          (174.19).inches, // FIRST's diagram has a typo (it says 147.19)
          (18.22).inches,
          Rotation3d()
        )
      ),
      AprilTag(7, Pose3d((40.45).inches, (108.19).inches, (18.22).inches, Rotation3d())),
      AprilTag(8, Pose3d((40.45).inches, (42.19).inches, (18.22).inches, Rotation3d()))
    )

  val wpilibAprilTags =
    if (Constants.Universal.REAL_FIELD) aprilTags.map { it.apriltagWpilib }
    else homeAprilTags.map { it.apriltagWpilib }

  val wpilibFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      wpilibAprilTags, fieldLength.inMeters, fieldWidth.inMeters
    )

  /**
   * Flips a translation to the correct side of the field based on the current alliance color. By
   * default, all translations and poses in [FieldConstants] are stored with the origin at the
   * rightmost point on the BLUE ALLIANCE wall.
   */
  fun allianceFlip(translation: Translation2d): Translation2d {
    return if (DriverStation.getAlliance() == Alliance.Red) {
      Translation2d(fieldLength - translation.x, translation.y)
    } else {
      translation
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color. By default,
   * all translations and poses in [FieldConstants] are stored with the origin at the rightmost
   * point on the BLUE ALLIANCE wall.
   */
  fun allianceFlip(pose: Pose2d): Pose2d {
    return if (DriverStation.getAlliance() == Alliance.Red) {
      Pose2d(fieldLength - pose.x, pose.y, Angle(-pose.rotation.cos, pose.rotation.sin))
    } else {
      pose
    }
  }

  fun getTagPose(id: Int): Pose3d? {
    return if (Constants.Universal.REAL_FIELD) aprilTags.firstOrNull { it.id == id }?.pose
    else homeAprilTags.firstOrNull { it.id == id }?.pose
  }

  // Dimensions for community and charging station, including the tape.
  object Community {
    // Region dimensions
    val innerX = 0.0.inches
    val midX = (132.375).inches // Tape to the left of charging station
    val outerX = (193.25.inches) // Tape to the right of charging station
    val leftY = 18.0.feet
    val midY = leftY - (59.39).inches + tapeWidth
    val rightY = 0.0.inches
    val regionCorners: Array<Translation2d> =
      arrayOf<Translation2d>(
        Translation2d(innerX, rightY),
        Translation2d(innerX, leftY),
        Translation2d(midX, leftY),
        Translation2d(midX, midY),
        Translation2d(outerX, midY),
        Translation2d(outerX, rightY)
      )

    // Charging station dimensions
    val chargingStationLength = (76.125).inches
    val chargingStationWidth = (97.25).inches
    val chargingStationOuterX = outerX - tapeWidth
    val chargingStationInnerX = chargingStationOuterX - chargingStationLength
    val chargingStationLeftY = midY - tapeWidth
    val chargingStationRightY = chargingStationLeftY - chargingStationWidth
    val chargingStationCorners: Array<Translation2d> =
      arrayOf<Translation2d>(
        Translation2d(chargingStationInnerX, chargingStationRightY),
        Translation2d(chargingStationInnerX, chargingStationLeftY),
        Translation2d(chargingStationOuterX, chargingStationRightY),
        Translation2d(chargingStationOuterX, chargingStationLeftY)
      )

    // Cable bump
    val cableBumpInnerX = innerX + Grids.outerX + (95.25).inches
    val cableBumpOuterX = cableBumpInnerX + (7).inches
    val cableBumpCorners: Array<Translation2d> =
      arrayOf<Translation2d>(
        Translation2d(cableBumpInnerX, 0.0.inches),
        Translation2d(cableBumpInnerX, chargingStationRightY),
        Translation2d(cableBumpOuterX, 0.0.inches),
        Translation2d(cableBumpOuterX, chargingStationRightY)
      )
  }

  // Dimensions for grids and nodes
  object Grids {
    // X layout
    val outerX = (54.25).inches
    val lowX = outerX - (14.25).inches / 2.0 // Centered when under cube nodes
    val midX = outerX - (22.75).inches
    val highX = outerX - (39.75).inches

    // Y layout
    const val nodeRowCount = 9
    val nodeFirstY = (20.19).inches
    val nodeSeparationY = (22.0).inches

    // Z layout
    val cubeEdgeHigh = (3.0).inches
    val highCubeZ = (48.5).inches - cubeEdgeHigh
    val midCubeZ = (34.5).inches - cubeEdgeHigh
    val highConeZ = (48.0).inches
    val midConeZ = (34.0).inches

    // Translations (all nodes in the same column/row have the same X/Y coordinate)
    val lowTranslations: Array<Translation2d?> = arrayOfNulls<Translation2d>(nodeRowCount)
    val midTranslations: Array<Translation2d?> = arrayOfNulls<Translation2d>(nodeRowCount)
    val mid3dTranslations: Array<Translation3d?> = arrayOfNulls<Translation3d>(nodeRowCount)
    val highTranslations: Array<Translation2d?> = arrayOfNulls<Translation2d>(nodeRowCount)
    val high3dTranslations: Array<Translation3d?> = arrayOfNulls<Translation3d>(nodeRowCount)

    // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
    val complexLowXCones = outerX - (16.0).inches / 2.0 // Centered X under cone nodes
    val complexLowXCubes = lowX // Centered X under cube nodes
    val complexLowOuterYOffset = nodeFirstY - (3.0).inches - (25.75).inches / 2.0
    val complexLowTranslations: Array<Translation2d> =
      arrayOf<Translation2d>(
        Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
        Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
        Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
        Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
        Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
        Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
        Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
        Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
        Translation2d(
          complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset
        )
      )

    init {
      for (i in 0 until nodeRowCount) {
        val isCube = i == 1 || i == 4 || i == 7
        lowTranslations[i] = Translation2d(lowX, nodeFirstY + nodeSeparationY * i)
        midTranslations[i] = Translation2d(midX, nodeFirstY + nodeSeparationY * i)
        mid3dTranslations[i] =
          Translation3d(
            midX, nodeFirstY + nodeSeparationY * i, if (isCube) midCubeZ else midConeZ
          )
        high3dTranslations[i] =
          Translation3d(
            highX, nodeFirstY + nodeSeparationY * i, if (isCube) highCubeZ else highConeZ
          )
        highTranslations[i] = Translation2d(highX, nodeFirstY + nodeSeparationY * i)
      }
    }
  }

  // Dimensions for loading zone and substations, including the tape
  object LoadingZone {
    // Region dimensions
    val width = (99.0).inches
    val innerX = fieldLength
    val midX = fieldLength - (132.25).inches
    val outerX = fieldLength - (264.25).inches
    val leftY = fieldWidth
    val midY = leftY - (50.5).inches
    val rightY = leftY - width
    val regionCorners: Array<Translation2d> =
      arrayOf<Translation2d>(
        Translation2d(
          midX, rightY
        ), // Start at lower left next to border with opponent community
        Translation2d(midX, midY),
        Translation2d(outerX, midY),
        Translation2d(outerX, leftY),
        Translation2d(innerX, leftY),
        Translation2d(innerX, rightY)
      )

    // Double substation dimensions
    val doubleSubstationLength = (14.0).inches
    val doubleSubstationX = innerX - doubleSubstationLength
    val doubleSubstationShelfZ = (37.375).inches

    // Single substation dimensions
    val singleSubstationWidth = (22.75).inches
    val singleSubstationLeftX = fieldLength - doubleSubstationLength - (88.77).inches
    val singleSubstationCenterX = singleSubstationLeftX + singleSubstationWidth / 2.0
    val singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth
    val singleSubstationTranslation: Translation2d = Translation2d(singleSubstationCenterX, leftY)
    val singleSubstationHeight = (38.0).inches
    val singleSubstationLowZ = (27.125).inches
    val singleSubstationCenterZ = singleSubstationLowZ + singleSubstationHeight / 2.0
    val singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight
  }

  // Locations of staged game pieces
  object StagingLocations {
    val centerOffsetX = (47.36).inches
    val positionX = fieldLength / 2.0 - (47.36).inches
    val firstY = (36.19).inches
    val separationY = (48.0).inches
    val translations: Array<Translation2d?> = arrayOfNulls<Translation2d>(4)

    init {
      for (i in translations.indices) {
        translations[i] = Translation2d(positionX, firstY + separationY * i)
      }
    }
  }
}
