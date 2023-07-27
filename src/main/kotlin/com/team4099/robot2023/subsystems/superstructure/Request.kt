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
      val finalVelocity: LinearVelocity = 0.0.inches.perSecond
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

/**
 * States
 * - Uninitialized
 * - Home Prep
 * - Home
 * - Idle (elevator fully retracted, manipulator fully retracted, ground intake stowed out/in)
 * - Intake cube prep
 * - Intake cube from ground
 * - Intake cube from ground clean up
 * - Intake cone prep
 * - Intake cone from ground
 * - Intake cone from ground clean up
 * - Intake game piece from double station prep
 * - Intake game piece from double substation
 * - Intake cube from single substation
 * - Intake cone from single substation
 * - Prep cube for scoring
 * - Prep cone for scoring
 * - Score cube
 * - Score cone
 *
 * Requests
 * - Intake cube from ground
 * - Intake cone from ground
 * - Intake game piece from double substation
 * - Intake cube from single substation
 * - Intake cone from single substation
 * - Score game piece on hybrid node
 * - Prep cube for scoring on high/mid node
 * - Prep cone for scoring on high/mid node
 * - Score cube on high/mid node
 * - Score cone on high/mid node
 * - Home elevator & manipulator
 * - Idle (stow ground intake, manipulator retracted, elevator all the way down)
 */
