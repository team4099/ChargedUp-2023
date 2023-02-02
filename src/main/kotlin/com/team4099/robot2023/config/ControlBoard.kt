package com.team4099.robot2023.config

import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad

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

  val runElevatorToHighNode = Trigger { operator.aButton }

  val openLoopExtend = Trigger { operator.bButton }

  val openLoopRetract = Trigger { operator.xButton }

  val characterizeElevator = Trigger { operator.yButton }
}
