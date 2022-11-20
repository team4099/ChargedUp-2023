package com.team4099.robot2022.config

import com.team4099.lib.joystick.XboxOneGamepad
import com.team4099.robot2022.config.constants.Constants
import edu.wpi.first.wpilibj2.command.button.Trigger

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {
  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis

  val robotOriented: Boolean
    get() = driver.leftShoulderButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  // buttons not final
  //  val runFeederIn = Trigger { operator.rightTriggerAxis > 0.5 }
  //  val runFeederOut = Trigger { operator.leftTriggerAxis > 0.5 }

  //  val resetBallCount = Trigger { operator.startButton && operator.selectButton }

  val openLoopExtend = Trigger { operator.aButton }
  val openLoopRetract = Trigger { operator.bButton }
}
