package com.team4099.robot2023.subsystems.elevator

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

object ElevatorIOSim : ElevatorIO {
  init {
    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    val m_mech2d = Mechanism2d(90.0, 90.0)
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
      gridHome.append(
        MechanismLigament2d("Grid Wall", 49.75, 180.0, 50.0, Color8Bit(Color.kWhite))
      )
    val dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37.0)
    val DSRamp =
      dsHome.append(
        MechanismLigament2d(
          "Double Substation Ramp", 13.75, 180.0, 10.0, Color8Bit(Color.kWhite)
        )
      )

    val m_bumper =
      gridHome.append(MechanismLigament2d("Bumper", 30.5, 0.0, 60.0, Color8Bit(Color.kRed)))

    SmartDashboard.putData("Arm Sim", m_mech2d)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {}
}
