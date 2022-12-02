package com.team4099.robot2022.subsystems.climber

import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.config.constants.PivotClimberConstants
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType

object PivotClimberIOReal : PivotClimberIO {

  private val pivotSolenoid =
    DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.PivotClimber.IN_PORT,
      Constants.PivotClimber.OUT_PORT
    )

  init {}

  override fun setPivotSolenoid(solenoidValue: DoubleSolenoid.Value) {
    pivotSolenoid.set(solenoidValue)
  }

  override fun updateInputs(inputs: PivotClimberIO.PivotClimberIOInputs) {
    inputs.isExtended =
      pivotSolenoid.get() == PivotClimberConstants.DesiredPivotStates.OUT.extendPosition
  }
}
