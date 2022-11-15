package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.inKilograms
import com.team4099.lib.units.base.inMeters
import com.team4099.robot2022.config.constants.ElevatorConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim

/** Elevator subsystem implementation using WPILIB simulation */
object ElevatorIOSim : ElevatorIO {
  val simulatedElevator: ElevatorSim =
    ElevatorSim(
      DCMotor.getNEO(2),
      ElevatorConstants.GEARING,
      ElevatorConstants.CARRIAGE_MASS.inKilograms,
      ElevatorConstants.DRUM_RADIUS.inMeters,
      ElevatorConstants.BOTTOM_HEIGHT_OF_TOP_OF_CARRIAGE.inMeters,
      ElevatorConstants.MAX_HEIGHT_OF_TOP_OF_CARRIAGE.inMeters,
      true
    )

  override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
    TODO()
  }
}
