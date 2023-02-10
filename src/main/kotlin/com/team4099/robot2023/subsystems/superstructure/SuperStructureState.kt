package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

// typealias GroundIntakeRequest = SuperStructureState.GroundIntakeStructure.GroundIntakeRequest
// typealias GroundIntakeState = SuperStructureState.GroundIntakeStructure.GroundIntakeState
// typealiasing for nested interfaces and sealed classes doesn't work
// https://youtrack.jetbrains.com/issue/KT-34281/Access-nested-classes-including-sealed-class-subclasses-through-typealias

sealed class SuperStructureState {

  sealed class GroundIntakeStructure : SuperStructureState() {

    sealed class GroundIntakeState : GroundIntakeStructure() {
      class Uninitialized : GroundIntakeState()
      class TargetingPosition : GroundIntakeState()
      class OpenLoop : GroundIntakeState()
    }

    // Implements SuperStructureRequest to ensure standardized structure
    sealed class GroundIntakeRequest : GroundIntakeStructure() {
      class TargetingPosition(val position: Angle) : GroundIntakeRequest()
      class OpenLoop(val voltage: ElectricalPotential) : GroundIntakeRequest()
    }

    override fun equals(other: Any?): Boolean {
      if (other is GroundIntakeState) {
        return other.javaClass.name == this.javaClass.name
      }
      return false
    }
  }
}
