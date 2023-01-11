package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.logging.TunableNumber
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.units.base.Length

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  val inputs = ElevatorIO.ElevatorInputs()
  val elevatorFeedForward =
    ElevatorFeedforward(
      ElevatorConstants.ELEVATOR_KS,
      ElevatorConstants.ELEVATOR_KG,
      ElevatorConstants.ELEVATOR_KV,
      ElevatorConstants.ELEVATOR_KA
    )

  private val kP = TunableNumber("Elevator/kP", ElevatorConstants.KP)
  private val kI = TunableNumber("Elevator/kI", ElevatorConstants.KI)
  private val kD = TunableNumber("Elevator/kD", ElevatorConstants.KD)

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition > ElevatorConstants.ELEVATOR_MAX_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition < ElevatorConstants.ELEVATOR_MAX_RETRACTION

  val currentState: ElevatorConstants.ActualElevatorStates
    get() {
      return when (inputs.elevatorPosition) {
        // TODO ("Someone check this")
        in (ElevatorConstants.DesiredElevatorStates.MIN_HEIGHT.height)..(
          ElevatorConstants
            .DesiredElevatorStates.MIN_HEIGHT
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.MIN_HEIGHT
        in (
          ElevatorConstants.DesiredElevatorStates.MIN_HEIGHT.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .GROUND_INTAKE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_MIN_AND_GROUND_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.GROUND_INTAKE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .GROUND_INTAKE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.GROUND_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.GROUND_INTAKE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .LOW_CUBE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_GROUND_INTAKE_AND_LOW_CUBE
        in (
          ElevatorConstants.DesiredElevatorStates.LOW_CUBE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .LOW_CUBE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.LOW_CUBE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.LOW_CUBE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .LOW_CONE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_LOW_CUBE_AND_LOW_CONE
        in (
          ElevatorConstants.DesiredElevatorStates.LOW_CONE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .LOW_CONE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.LOW_CONE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.LOW_CONE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MID_CUBE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_LOW_CONE_AND_MID_CUBE
        in (
          ElevatorConstants.DesiredElevatorStates.MID_CUBE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MID_CUBE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.MID_CUBE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.MID_CUBE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MID_CONE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_MID_CUBE_AND_MID_CONE
        in (
          ElevatorConstants.DesiredElevatorStates.MID_CONE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MID_CONE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.MID_CONE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.MID_CONE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .CUBE_SHELF_INTAKE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_MID_CONE_AND_CUBE_SHELF_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.CUBE_SHELF_INTAKE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .CUBE_SHELF_INTAKE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.CUBE_SHELF_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.CUBE_SHELF_INTAKE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .CONE_SHELF_INTAKE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_CUBE_SHELF_INTAKE_AND_CONE_SHELF_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.CONE_SHELF_INTAKE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .CONE_SHELF_INTAKE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.CONE_SHELF_INTAKE
        in (
          ElevatorConstants.DesiredElevatorStates.CONE_SHELF_INTAKE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .HIGH_CUBE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_CONE_SHELF_INTAKE_AND_HIGH_CUBE
        in (
          ElevatorConstants.DesiredElevatorStates.HIGH_CUBE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .HIGH_CUBE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.HIGH_CUBE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.HIGH_CUBE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .HIGH_CONE_SCORE
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_HIGH_CUBE_AND_HIGH_CONE
        in (
          ElevatorConstants.DesiredElevatorStates.HIGH_CONE_SCORE.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .HIGH_CONE_SCORE
            .height + ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.HIGH_CONE_SCORE
        in (
          ElevatorConstants.DesiredElevatorStates.HIGH_CONE_SCORE.height +
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MAX_HEIGHT
            .height - ElevatorConstants.elevatorTolerance
          ) ->
          ElevatorConstants.ActualElevatorStates.BETWEEN_HIGH_CONE_AND_MAX_HEIGHT
        in (
          ElevatorConstants.DesiredElevatorStates.MAX_HEIGHT.height -
            ElevatorConstants.elevatorTolerance
          )..(
          ElevatorConstants.DesiredElevatorStates
            .MAX_HEIGHT
            .height
          ) -> ElevatorConstants.ActualElevatorStates.MAX_HEIGHT
        else -> {
          ElevatorConstants.ActualElevatorStates.DUMMY
        }
      }
    }

  init {}

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  fun setOpenLoop(percentOutput: Double) {
    io.setOpenLoop(percentOutput)

    if ((forwardLimitReached || reverseLimitReached) && percentOutput != 0.0) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }
  }

  fun setPosition(height: Length) {
    io.setPosition(height)
  }
}
