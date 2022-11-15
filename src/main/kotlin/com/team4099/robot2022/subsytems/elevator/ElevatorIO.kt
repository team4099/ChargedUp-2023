package com.team4099.robot2022.subsytems.elevator

import org.littletonrobotics.junction.AutoLog

interface ElevatorIO {
  @AutoLog class ElevatorIOInputs

  fun updateInputs(inputs: ElevatorIOInputs) {}
}
