package com.team4099.robot2023.subsystems.motorchecker

import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.Temperature

data class MotorCollection(
  val motorCollection: List<Motor<MotorType>>,
  val baseCurrentLimit: Current,
  val firstStageTemperatureLimit: Temperature,
  val firstStageCurrentLimit: Current,
  val motorShutDownThreshold: Temperature
) {

  val maxMotorTemperature: Temperature
    get() = motorCollection.maxOf { it.temperature }

  fun setCurrentLimit(limit: Current): Boolean {
    var retValue = true
    for (motor in motorCollection) {
      retValue = retValue and motor.setCurrentLimit(limit)
    }

    return retValue
  }

  val currentLimitStage: CURRENT_STAGE_LIMIT
    get() =
      if (motorCollection.all { it.currentLimitStage == motorCollection[0].currentLimitStage })
        motorCollection[0].currentLimitStage
      else CURRENT_STAGE_LIMIT.NONE
}
