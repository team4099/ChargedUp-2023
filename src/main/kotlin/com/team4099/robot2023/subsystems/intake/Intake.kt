package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Intake(val io: IntakeIO) : SubsystemBase() {
  val inputs = IntakeIO.IntakeIOInputs()

  var lastIntakeRunTime = Clock.fpgaTime

  var rollerState = IntakeConstants.ROLLER_STATE.IDLE
    set(state) {
      io.setRollerPower(state.power)
      if (state == IntakeConstants.ROLLER_STATE.INTAKE){
        lastIntakeRunTime = Clock.fpgaTime
      }
      field = state
    }

  init {}

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Intake", inputs)
  }
}
