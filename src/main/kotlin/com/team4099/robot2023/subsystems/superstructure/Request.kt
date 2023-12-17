package com.team4099.robot2023.subsystems.superstructure

import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.perSecond

// typealias GroundIntakeRequest = SuperStructureState.GroundIntakeStructure.GroundIntakeRequest
// typealias GroundIntakeState = SuperStructureState.GroundIntakeStructure.GroundIntakeState
// typealiasing for nested interfaces and sealed classes doesn't work
// https://youtrack.jetbrains.com/issue/KT-34281/Access-nested-classes-including-sealed-class-subclasses-through-typealias

sealed interface Request {

  sealed interface SuperstructureRequest : Request {
    class Idle() : SuperstructureRequest
    class Home() : SuperstructureRequest

    class GroundIntakeCube() : SuperstructureRequest
    class GroundIntakeCone() : SuperstructureRequest

    class EjectGamePiece() : SuperstructureRequest

    class DoubleSubstationIntakePrep(val gamePiece: GamePiece) : SuperstructureRequest
    class SingleSubstationIntakePrep(val gamePiece: GamePiece) : SuperstructureRequest

    class DoubleSubstationIntake() : SuperstructureRequest
    class SingleSubstationIntake(val gamePiece: GamePiece) : SuperstructureRequest

    class PrepScore(val gamePiece: GamePiece, val nodeTier: NodeTier) : SuperstructureRequest

    class Score() : SuperstructureRequest

    class Tuning() : SuperstructureRequest
  }

  // Implements RequestStructure to ensure standardized structure
  sealed interface ManipulatorRequest : Request {
    class TargetingPosition(val position: Length, val rollerVoltage: ElectricalPotential) :
      ManipulatorRequest
    class OpenLoop(val voltage: ElectricalPotential, val rollerVoltage: ElectricalPotential) :
      ManipulatorRequest
    class Home() : ManipulatorRequest
  }

  sealed interface ElevatorRequest : Request {
    class TargetingPosition(
      val position: Length,
      val finalVelocity: LinearVelocity = 0.0.inches.perSecond,
      val canContinueBuffer: Length = 5.0.inches
    ) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home : ElevatorRequest
  }

  sealed interface GroundIntakeRequest : Request {
    class TargetingPosition(val position: Angle, val rollerVoltage: ElectricalPotential) :
      GroundIntakeRequest
    class OpenLoop(val voltage: ElectricalPotential, val rollerVoltage: ElectricalPotential) :
      GroundIntakeRequest
    class ZeroArm() : GroundIntakeRequest
  }

  sealed interface DrivetrainRequest : Request {
    class OpenLoop(
      val angularVelocity: AngularVelocity,
      val driveVector: Pair<LinearVelocity, LinearVelocity>,
      val fieldOriented: Boolean = true
    ) : DrivetrainRequest
    class ClosedLoop(
      val chassisSpeeds: ChassisSpeeds,
      val chassisAccels: ChassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    ) : DrivetrainRequest
    class ZeroSensors : DrivetrainRequest
    class Idle : DrivetrainRequest
  }
}
