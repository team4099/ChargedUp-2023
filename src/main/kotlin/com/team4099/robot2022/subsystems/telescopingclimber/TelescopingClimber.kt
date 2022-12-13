package com.team4099.robot2022.subsystems.climber

import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.TelescopingClimberConstants
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase

class TelescopingClimber(val io: TelescopingClimberIO) : SubsystemBase() {
  val inputs = TelescopingClimberIO.TelescopingClimberIOInputs()

  val loadedFeedForward: ElevatorFeedforward =
    ElevatorFeedforward(
      TelescopingClimberConstants.LOAD_KS.inVolts,
      TelescopingClimberConstants.LOAD_KG.inVolts,
      (1.meters.perSecond * TelescopingClimberConstants.LOAD_KV).inVolts,
      (1.meters.perSecond.perSecond * TelescopingClimberConstants.LOAD_KA).inVolts
    )

  val noLoadFeedForward: ElevatorFeedforward =
    ElevatorFeedforward(
      TelescopingClimberConstants.NO_LOAD_KS.inVolts,
      TelescopingClimberConstants.NO_LOAD_KG.inVolts,
      (1.meters.perSecond * TelescopingClimberConstants.NO_LOAD_KV).inVolts,
      (1.meters.perSecond.perSecond * TelescopingClimberConstants.NO_LOAD_KA).inVolts
    )
}
