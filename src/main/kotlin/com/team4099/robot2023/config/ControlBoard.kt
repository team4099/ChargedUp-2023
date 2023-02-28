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
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis

  val robotOriented: Boolean
    get() = driver.leftShoulderButton

  val autoLevel = Trigger { driver.aButton }

  val pickupFromSubstation = Trigger { driver.bButton }

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  val extendArm = Trigger { operator.aButton }

  val retractArm = Trigger { operator.bButton }

  val setArmPositionToShelfIntake = Trigger { operator.xButton }

  // val armCharacterization = Trigger { operator.yButton }

  /*
  val intakeCone = Trigger { operator.aButton }

  val intakeCube = Trigger { operator.bButton }

  val outtakeCone = Trigger { operator.yButton }

  val outtakeCube = Trigger { operator.xButton }

   */
  val runElevatorToHighNode = Trigger { operator.aButton }

  val openLoopExtend = Trigger { operator.bButton }

  val openLoopRetract = Trigger { operator.xButton }

  val characterizeElevator = Trigger { operator.yButton }

  val extendIntake = Trigger { technician.aButton }
  val retractIntake = Trigger { technician.bButton }
  val characterizeIntake = Trigger { technician.xButton }

  // for testing
  val setArmCommand = Trigger { technician.yButton }
}
