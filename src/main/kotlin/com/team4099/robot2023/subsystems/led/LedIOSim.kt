package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LedConstants.LEDMode
import com.team4099.robot2023.config.constants.LedConstants.SimMode
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit



object LedIOSim : LedIO {
  private val ledWidget = Mechanism2d(400.0, 400.0, Color8Bit(Color.kWhite))
  private var state: LEDMode = LEDMode.IDLE

  override fun updateInputs(inputs: LedIO.LedIOInputs) {
    inputs.ledState = state.name
  }

  override fun setState(newState: LEDMode) {
    state = newState

    when (state){
      LEDMode.IDLE -> setWidget(SimMode.IDLE)
      LEDMode.ALERT -> setWidget(SimMode.ALERT)
      LEDMode.ITEM -> setWidget(SimMode.ITEM)
      LEDMode.POS_LEVELED -> setWidget(SimMode.POS_LEVELED)
      LEDMode.NEG_LEVELED -> setWidget(SimMode.NEG_LEVELED)
      LEDMode.OUTTAKE -> setWidget(SimMode.OUTTAKE)
      LEDMode.AUTO -> setWidget(SimMode.AUTO)
      LEDMode.TELEOP -> setWidget(SimMode.TELEOP)
      LEDMode.DISABLED -> setWidget(SimMode.DISABLED)
    }
  }

  private fun setWidget(color: SimMode){
    ledWidget.setBackgroundColor(color.color)
  }



}
