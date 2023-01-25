package com.team4099.robot2023.commands.manipulator

import com.team4099.robot2023.subsystems.manipulator.Manipulator
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ArmCharacterizationCommand(val manipulator: Manipulator) : CommandBase() {
  init {
    addRequirements(manipulator)
  }

  var hasMoved = false
  var appliedVolts = 0.volts
  var step = 0.001.volts

  override fun execute() {
    manipulator.setArmVoltage(appliedVolts)

    if (manipulator.inputs.armVelocity < 0.meters.perSecond) {
      hasMoved = true
      println(appliedVolts.inVolts)
    }
    appliedVolts += step
  }

  override fun isFinished(): Boolean {
    return hasMoved
  }
}
