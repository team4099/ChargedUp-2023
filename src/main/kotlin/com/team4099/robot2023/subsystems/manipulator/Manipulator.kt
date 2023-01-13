package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds

class Manipulator(val io: ManipulatorIO) : SubsystemBase() {
  val inputs = ManipulatorIO.IntakeIOInputs()

  var lastIntakeRunTime = Clock.fpgaTime

  var rollerState = ManipulatorConstants.RollerState.IDLE
    set(state) {
      io.setRollerPower(state.speed)
      if (state == ManipulatorConstants.RollerState.CONE_IN ||
        state == ManipulatorConstants.RollerState.CUBE_IN
      ) {
        lastIntakeRunTime = Clock.fpgaTime
      }
      field = state
    }

  val intakingGamePiece: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.INTAKE_CURRENT_THRESHOLD &&
        (
          rollerState == ManipulatorConstants.RollerState.CONE_IN ||
            rollerState == ManipulatorConstants.RollerState.CONE_IN
          ) &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
        ManipulatorConstants.INTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  val outtakingGamePiece: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.OUTAKE_CURRENT_THRESHOLD &&
        (
          rollerState == ManipulatorConstants.RollerState.CONE_OUT ||
            rollerState == ManipulatorConstants.RollerState.CUBE_OUT
          ) &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
        ManipulatorConstants.OUTTAKING_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  var lastIntakeSpikeTime = Clock.fpgaTime

  init {
    // setter isn't called on initialization
    rollerState = rollerState
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (intakingGamePiece || outtakingGamePiece) {
      lastIntakeSpikeTime = Clock.fpgaTime
    }

    Logger.getInstance().processInputs("Intake", inputs)
    Logger.getInstance().recordOutput("Intake/rollerState", rollerState.name)
    Logger.getInstance().recordOutput("Intake/intakingBall", intakingGamePiece)
    Logger.getInstance().recordOutput("Intake/outtakingBall", outtakingGamePiece)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeRunTime.inSeconds)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeSpikeTime.inSeconds)
  }
}
