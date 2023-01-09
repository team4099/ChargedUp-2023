package com.team4099.robot2023.subsystems.led

<<<<<<< HEAD
import com.team4099.robot2023.config.constants.LedConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Led(val io: LedIO) : SubsystemBase() {
  var inputs = LedIO.LedIOInputs()
  var state = LedConstants.LEDMode.IDLE
    set(value) {
      io.setState(value)
      field = value
    }

  init {
    state = state
  }

  override fun periodic() {
    io.updateInputs(inputs)
    Logger.getInstance().processInputs("LED", inputs)
    Logger.getInstance().recordOutput("LED/state", state.name)
  }
=======
class Led {
>>>>>>> f6d2f8f (added files and some code)
}
