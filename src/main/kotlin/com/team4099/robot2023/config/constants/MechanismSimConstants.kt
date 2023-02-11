package com.team4099.robot2023.config.constants

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.inDegrees

object MechanismSimConstants {

  val m_mech2d = Mechanism2d(90.0, 90.0)

  // field stuff
  val midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0.0)
  val MidNode =
    midNodeHome.append(
      MechanismLigament2d("Mid Cone Node", 34.0, 90.0, 10.0, Color8Bit(Color.kWhite))
    )
  val highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0.0)
  val HighNode =
    highNodeHome.append(
      MechanismLigament2d("High Cone Node", 46.0, 90.0, 10.0, Color8Bit(Color.kWhite))
    )
  val gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0.0)
  val GridNode =
    gridHome.append(MechanismLigament2d("Grid Wall", 49.75, 180.0, 50.0, Color8Bit(Color.kWhite)))
  val dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37.0)
  val DSRamp =
    dsHome.append(
      MechanismLigament2d(
        "Double Substation Ramp", 13.75, 180.0, 10.0, Color8Bit(Color.kWhite)
      )
    )

  // bumper stuff

  val bumperX = 49.75.inches
  val bumperY = 0.0.inches
  val bumperHeight = 6.inches
  val bumperWidth = 30.5.inches

  val bumperHome = m_mech2d.getRoot("Bumper Home", bumperX.inInches, bumperY.inInches)

  val m_bumper =
    bumperHome.append(MechanismLigament2d("Bumper", 30.5, 0.0, 60.0, Color8Bit(Color.kRed)))

  val elevatorRelativeXPosition = 27.inches
  val elevatorRelativeYPosition = bumperHeight
  val elevatorAbsoluteXPosition = elevatorRelativeXPosition + bumperX
  val elevatorAbsoluteYPosition = elevatorRelativeYPosition + bumperY

  // elevator stuff
  val firstStageAttachment =
    m_mech2d.getRoot(
      "First Stage Attachment",
      elevatorAbsoluteXPosition.inInches,
      elevatorAbsoluteYPosition.inInches
    )
  val secondStageAttachment =
    m_mech2d.getRoot(
      "Second Stage Attachment",
      elevatorAbsoluteXPosition.inInches,
      elevatorAbsoluteYPosition.inInches
    )
  val carriageAttachment =
    m_mech2d.getRoot(
      "Carriage Attachment",
      elevatorAbsoluteXPosition.inInches,
      elevatorAbsoluteYPosition.inInches
    )

  val m_elevatorFirstStage =
    firstStageAttachment.append(
      MechanismLigament2d(
        "Elevator First Stage",
        ElevatorConstants.FIRST_STAGE_HEIHT.inInches,
        180.0 - ElevatorConstants.ELEVATOR_ANGLE.inDegrees,
        15.0,
        Color8Bit(Color.kOrange)
      )
    )

  val m_elevatorSecondState =
    secondStageAttachment.append(
      MechanismLigament2d(
        "Elevator Second Stage",
        ElevatorConstants.SECOND_STAGE_HEIGHT.inInches,
        180.0 - ElevatorConstants.ELEVATOR_ANGLE.inDegrees,
        15.0,
        Color8Bit(Color.kYellow)
      )
    )

  val m_carriage =
    carriageAttachment.append(
      MechanismLigament2d("Elevator Carriage", 10.0, 180.0, 10.0, Color8Bit(Color.kGreen))
    )
}
