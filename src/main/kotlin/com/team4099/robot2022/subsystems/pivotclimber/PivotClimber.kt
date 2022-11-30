package com.team4099.robot2022.subsystems.climber

import com.team4099.robot2022.config.constants.PivotClimberConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class PivotClimber(val io: PivotClimberIO) : SubsystemBase() {
  val inputs = PivotClimberIO.PivotClimberIOInputs()

  val desiredState = PivotClimberConstants.DesiredPivotStates.IN

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Pivot Climber", inputs)
    Logger.getInstance().recordOutput("PivotClimberDesiredState", desiredState.name)
  }
}
