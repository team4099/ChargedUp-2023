package com.team4099.robot2023.subsystems.gameboy.objective

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.NodeTier
import com.team4099.robot2023.config.constants.Substation

data class Objective(
  val nodeColumn: Int = -1,
  val nodeTier: NodeTier = NodeTier.NONE,
  var substation: Substation = Substation.NONE
)

fun Objective.isValidObjective(): Boolean {
  val substationSelected = this.substation != Constants.Universal.Substation.NONE
  val scoringPositionSelected =
    this.nodeColumn != -1 && this.nodeTier != Constants.Universal.NodeTier.NONE

  return substationSelected xor scoringPositionSelected
}

fun Objective.isConeNode(): Boolean {
  return this.nodeTier != Constants.Universal.NodeTier.HYBRID &&
    this.nodeColumn in listOf(2, 3, 5, 6, 8)
}
