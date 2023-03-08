package com.team4099.robot2023.subsystems.gameboy

import com.team4099.robot2023.subsystems.gameboy.objective.Objective
import org.littletonrobotics.junction.Logger

class GameBoy(val io: GameboyIO) {

  val inputs = GameboyIO.GameboyIOInputs()
  var objective: Objective = Objective()

  fun periodic() {
    io.updateInputs(inputs)
    Logger.getInstance().processInputs("Gameboy", inputs)

    objective = inputs.objective
    io.setSelected(objective)
  }
}
