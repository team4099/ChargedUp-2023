package com.team4099.lib.joystick

/** Represents a generic controller which expresses a specific set of controls. */
interface Gamepad {
  val leftXAxis: Double
  val leftYAxis: Double
  val leftTriggerAxis: Double
  val rightTriggerAxis: Double
  val rightXAxis: Double
  val rightYAxis: Double
  val aButton: Boolean
  val bButton: Boolean
  val xButton: Boolean
  val yButton: Boolean
  val leftJoystickButton: Boolean
  val rightJoystickButton: Boolean
  val leftShoulderButton: Boolean
  val rightShoulderButton: Boolean
  val dPadUp: Boolean
  val dPadDown: Boolean
  val dPadLeft: Boolean
  val dPadRight: Boolean
  val startButton: Boolean
  val selectButton: Boolean
}
