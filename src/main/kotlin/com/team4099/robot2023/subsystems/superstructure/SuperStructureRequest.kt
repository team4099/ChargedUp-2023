package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

// typealias GroundIntakeRequest = SuperStructureState.GroundIntakeStructure.GroundIntakeRequest
// typealias GroundIntakeState = SuperStructureState.GroundIntakeStructure.GroundIntakeState
// typealiasing for nested interfaces and sealed classes doesn't work
// https://youtrack.jetbrains.com/issue/KT-34281/Access-nested-classes-including-sealed-class-subclasses-through-typealias

sealed interface SuperStructureRequest {
  // Implements SuperStructureRequest to ensure standardized structure
  sealed interface GroundIntakeRequest : SuperStructureRequest {
    class TargetingPosition(val position: Angle) : GroundIntakeRequest
    class OpenLoop(val voltage: ElectricalPotential) : GroundIntakeRequest
  }
}
