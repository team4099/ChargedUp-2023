package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.derived.volts
import com.team4099.lib.units.inMetersPerSecond
import com.team4099.lib.units.inMetersPerSecondPerSecond
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.config.constants.ElevatorConstants
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
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

  var trapezoidProfileConstrants: TrapezoidProfile.Constraints =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY.inMetersPerSecond,
      ElevatorConstants.MAX_ACCELERATION.inMetersPerSecondPerSecond
    )

  var elevatorSetpoint: TrapezoidProfile.State =
    TrapezoidProfile.State(inputs.position.inMeters, inputs.velocity.inMetersPerSecond)

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Elevator", inputs)
  }

  /**
   * Open loop control for the elevator motors
   * @param percentOutput Ratio that represents the amount of applied power
   */
  fun setOpenLoop(percentOutput: Double) {
    if (extensionLimitReached && percentOutput > 0.0) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }

    if (retractionLimitReached && percentOutput < 0.0) {
      io.setOpenLoop(0.0)
    } else {
      io.setOpenLoop(percentOutput)
    }
  }

  fun setPosition(setpoint: TrapezoidProfile.State) {
    // acceleration = dv/dt (where dt = 0.02 seconds)
    val acceleration =
      ((setpoint.velocity - elevatorSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME.inSeconds).meters.perSecond.perSecond

    elevatorSetpoint = setpoint

    val desiredFeedforward: ElectricalPotential =
      elevatorFeedforward.calculate(setpoint.velocity, acceleration.inMetersPerSecondPerSecond)
        .volts
    io.setPosition(setpoint.position.meters, desiredFeedforward)

    Logger.getInstance().recordOutput("Elevator/feedForwardVolts", desiredFeedforward.inVolts)
  }
}
