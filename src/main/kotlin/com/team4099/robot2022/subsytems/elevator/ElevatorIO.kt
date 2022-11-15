package com.team4099.robot2022.subsytems.elevator

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.inAmperes
import com.team4099.lib.units.base.inInches
import com.team4099.lib.units.base.inches
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.inInchesPerSecond
import com.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/** Interface for all hardware implementations for the Elevator subsystem */
interface ElevatorIO {
  /** Defines and logs all inputs that are required for all hardware implementations. */
  class ElevatorIOInputs : LoggableInputs {

    var position = 0.0.inches
    var velocity = 0.0.inches.perSecond
    var leaderSupplyCurrentDraw = 0.0.amps
    var followerSupplyCurrentDraw = 0.0.amps

    override fun toLog(table: LogTable?) {
      table?.put("positionInches", position.inInches)
      table?.put("velocityInchesPerSecond", velocity.inInchesPerSecond)
      table?.put("leaderSupplyCurrentDrawAmps", leaderSupplyCurrentDraw.inAmperes)
      table?.put("followerSupplyCurrentDraw", followerSupplyCurrentDraw.inAmperes)
    }

    override fun fromLog(table: LogTable?) {
      table?.getDouble("positionInches", position.inInches)?.let { position = it.inches }
      table?.getDouble("velocityInchesPerSecond", velocity.inInchesPerSecond)?.let {
        velocity = it.inches.perSecond
      }
      table?.getDouble("leaderSupplyCurrentDrawAmps", leaderSupplyCurrentDraw.inAmperes)?.let {
        leaderSupplyCurrentDraw = it.amps
      }
      table?.getDouble("followerSupplyCurrentDraw", followerSupplyCurrentDraw.inAmperes)?.let {
        followerSupplyCurrentDraw = it.amps
      }
    }
  }

  /**
   * Method to be implemented to update data encapsulated by `inputs` object
   * @param inputs Input data retrieved from hardware implementation and used by subsystem logic
   * class
   */
  fun updateInputs(inputs: ElevatorIOInputs) {}

  /**
   * Runs elevator motors at a specified power
   * @param percentOutput Ratio that represents the amount of applied power
   */
  fun setOpenLoop(percentOutput: Double) {}

  /**
   * Sets the position of the mechanism to a desired height
   * @param height Desired height
   * @param feedforward Applied voltage to elevator (at 12v nominal) during a loop cycle
   */
  fun setPosition(height: Length, feedforward: ElectricalPotential) {}
}
