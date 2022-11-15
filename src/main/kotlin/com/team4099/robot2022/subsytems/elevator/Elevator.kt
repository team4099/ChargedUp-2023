package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.ElevatorConstants
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Elevator(val io: ElevatorIO) : SubsystemBase() {
  // Initializing inputs object from which all IO data can be read from
  val inputs = ElevatorIO.ElevatorIOInputs()

  val elevatorFeedforward: ElevatorFeedforward =
    ElevatorFeedforward(
      ElevatorConstants.ELEVATOR_KS.inVolts,
      ElevatorConstants.ELEVATOR_KG.inVolts,
      (1.meters.perSecond * ElevatorConstants.ELEVATOR_KV).inVolts,
      (1.meters.perSecond.perSecond * ElevatorConstants.ELEVATOR_KA).inVolts
    )

  val extensionLimitReached: Boolean
    get() = inputs.position > ElevatorConstants.elevatorMaxExtension

  val retractionLimitReached: Boolean
    get() = inputs.position > ElevatorConstants.elevatorMinExtension

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)
  }
}
