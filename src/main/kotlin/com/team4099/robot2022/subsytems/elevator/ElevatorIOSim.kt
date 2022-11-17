package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.inKilograms
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.perSecond
import com.team4099.robot2022.config.constants.ElevatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim

/** Elevator subsystem implementation using WPILIB simulation */
object ElevatorIOSim : ElevatorIO {

  private val elevatorGearbox: DCMotor = DCMotor.getNEO(2)

  val simulatedElevator: ElevatorSim =
    ElevatorSim(
      elevatorGearbox,
      ElevatorConstants.GEARING,
      ElevatorConstants.CARRIAGE_MASS.inKilograms,
      ElevatorConstants.DRUM_RADIUS.inMeters,
      ElevatorConstants.BOTTOM_HEIGHT_OF_TOP_OF_CARRIAGE.inMeters,
      ElevatorConstants.MAX_HEIGHT_OF_TOP_OF_CARRIAGE.inMeters,
      true
    )

  override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
    inputs.position = simulatedElevator.positionMeters.meters
    inputs.velocity = simulatedElevator.velocityMetersPerSecond.meters.perSecond

    // WPILIB multiplies current draws by `numMotors`. To get the draw for each theoretical motor
    // you divide by `numMotors`
    inputs.leaderSupplyCurrentDraw = (simulatedElevator.currentDrawAmps / 2).amps
    inputs.followerSupplyCurrentDraw = (simulatedElevator.currentDrawAmps / 2).amps
  }
}
