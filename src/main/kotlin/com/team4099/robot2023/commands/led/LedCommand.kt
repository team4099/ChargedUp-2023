package com.team4099.robot2023.commands.led

import com.team4099.robot2023.subsystems.led.Led
import edu.wpi.first.wpilibj2.command.CommandBase

class LedCommand(val led: Led) : CommandBase() {

  init {
    addRequirements(led)
  }

  override fun execute() {
    /* led.state =
        when (Robot.isDisabled) {
          LedConstants.LEDMode.DISABLED
        }
    */
  }
}
