package com.team4099.robot2023.subsystems.gameboy

import com.team4099.robot2023.subsystems.gameboy.objective.Objective
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GameboyIO {
  class GameboyIOInputs : LoggableInputs {
    var objective = Objective()

    override fun toLog(table: LogTable?) {
      table?.put("Gameboy/selectedRow", objective.nodeColumn.toDouble())
      table?.put("Gameboy/selectedNodeTier", objective.nodeTier.name)
      table?.put("Gameboy/selectedSubstation", objective.substation.name)
    }

    override fun fromLog(table: LogTable?) {
      // if you wanna log replay this, don't <3
    }
  }

  fun updateInputs(inputs: GameboyIOInputs) {}

  fun setSelected(objective: Objective) {}
}
