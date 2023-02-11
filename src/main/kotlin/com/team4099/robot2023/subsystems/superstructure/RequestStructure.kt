package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.perSecond

// typealias GroundIntakeRequest = SuperStructureState.GroundIntakeStructure.GroundIntakeRequest
// typealias GroundIntakeState = SuperStructureState.GroundIntakeStructure.GroundIntakeState
// typealiasing for nested interfaces and sealed classes doesn't work
// https://youtrack.jetbrains.com/issue/KT-34281/Access-nested-classes-including-sealed-class-subclasses-through-typealias

sealed interface RequestStructure {

  // Implements RequestStructure to ensure standardized structure
  sealed interface ElevatorRequest : RequestStructure {
    class TargetingPosition(
      val position: Length,
      val finalVelocity: LinearVelocity = 0.0.inches.perSecond
    ) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home : ElevatorRequest
  }
}
