package com.team4099.lib.joystick

import edu.wpi.first.wpilibj.Joystick

/**
 * Controller Settings for Correct Mappings ---------------------------------------- X Emulation
 * Mode (switch on back = X) Flight Mode (Mode Light = Off)
 */
class XboxOneGamepad(port: Int) : Joystick(port), Gamepad {
  override val leftXAxis: Double
    get() = this.getRawAxis(0)

  override val leftYAxis: Double
    get() = this.getRawAxis(1)

  override val leftTriggerAxis: Double
    get() = this.getRawAxis(2)

  override val rightTriggerAxis: Double
    get() = this.getRawAxis(3)

  override val rightXAxis: Double
    get() = this.getRawAxis(4)

  override val rightYAxis: Double
    get() = this.getRawAxis(5)

  override val aButton: Boolean
    get() = this.getRawButton(1)

  override val bButton: Boolean
    get() = this.getRawButton(2)

  override val xButton: Boolean
    get() = this.getRawButton(3)

  override val yButton: Boolean
    get() = this.getRawButton(4)

  override val leftJoystickButton: Boolean
    get() = this.getRawButton(6)

  override val rightJoystickButton: Boolean
    get() = this.getRawButton(7)

  override val leftShoulderButton: Boolean
    get() = this.getRawButton(5)

  override val rightShoulderButton: Boolean
    get() = this.getRawButton(6)

  override val startButton: Boolean
    get() = this.getRawButton(7)

  override val selectButton: Boolean
    get() = this.getRawButton(8)

  override val dPadUp: Boolean
    get() = this.pov == 0 || this.pov == 360

  override val dPadDown: Boolean
    get() = this.pov == 180

  override val dPadLeft: Boolean
    get() = this.pov == 270

  override val dPadRight: Boolean
    get() = this.pov == 90
}
