package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds

class Intake(val io: IntakeIO) : SubsystemBase() {
  val inputs = IntakeIO.IntakeIOInputs()

  var lastIntakeRunTime = Clock.fpgaTime

  var grabberState = IntakeConstants.GrabberState.IDLE
    set(state) {
      io.setGrabberPower(state.speed)
      if (state == IntakeConstants.GrabberState.IN) {
        lastIntakeRunTime = Clock.fpgaTime
      }
      field = state
    }

  val intakingGamePiece: Boolean
    get() {
      return inputs.grabberStatorCurrent >= IntakeConstants.INTAKE_CURRENT_THRESHOLD &&
        grabberState == IntakeConstants.GrabberState.IN &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
        IntakeConstants.INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  val outtakingGamePiece: Boolean
    get() {
      return inputs.grabberStatorCurrent >= IntakeConstants.OUTAKE_CURRENT_THRESHOLD &&
        grabberState == IntakeConstants.GrabberState.OUT &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
        IntakeConstants.OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  var lastIntakeSpikeTime = Clock.fpgaTime

  init {
    // setter isn't called on initialization
    grabberState = grabberState
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (intakingGamePiece || outtakingGamePiece) {
      lastIntakeSpikeTime = Clock.fpgaTime
    }

    Logger.getInstance().processInputs("Intake", inputs)
    Logger.getInstance().recordOutput("Intake/rollerState", grabberState.name)
    Logger.getInstance().recordOutput("Intake/intakingBall", intakingGamePiece)
    Logger.getInstance().recordOutput("Intake/outtakingBall", outtakingGamePiece)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeRunTime.inSeconds)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeSpikeTime.inSeconds)
  }
}
