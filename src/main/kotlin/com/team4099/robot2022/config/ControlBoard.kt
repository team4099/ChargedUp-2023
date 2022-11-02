package com.team4099.robot2022.config

import com.team4099.lib.joystick.XboxOneGamepad
import com.team4099.robot2022.config.constants.Constants
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger

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

  val runIntake = Trigger { operator.aButton }

  // Shooter triggers
  // supposed to be yButton
  val startShooter = Trigger { operator.yButton }
  val startShooterLower = Trigger { operator.xButton }
  // supposed to be bButton
  val outTake = Trigger { operator.bButton }
  //  val shooterUnjam = Trigger { operator.selectButton }
  //  val alignRobot = Trigger { operator.yButton }

  val extendTelescoping = Trigger { operator.dPadUp }
  val retractTelescoping = Trigger { operator.dPadDown }

  val extendPivot = Trigger { operator.dPadRight }
  val retractPivot = Trigger { operator.dPadLeft }

  val startClimbSequence = Trigger { operator.startButton }
  val fixTelescopingRung = Trigger { operator.selectButton }

  val leftSpoolDown = Trigger { operator.leftShoulderButton }
  val rightSpoolDown = Trigger { operator.rightShoulderButton }

  val leftSpoolUp = Trigger { operator.leftTriggerAxis > 0.5 }
  val rightSpoolUp = Trigger { operator.rightTriggerAxis > 0.5 }

  fun logDriverController() {
    Logger.getInstance().recordOutput("DriverController/leftXAxis", driver.leftXAxis)
    Logger.getInstance().recordOutput("DriverController/rightXAxis", driver.rightXAxis)
    Logger.getInstance().recordOutput("DriverController/leftYAxis", driver.leftYAxis)
    Logger.getInstance()
      .recordOutput("DriverController/leftShoulderButton", driver.leftShoulderButton)
    Logger.getInstance().recordOutput("DriverController/startButton", driver.startButton)
    Logger.getInstance().recordOutput("DriverController/selectButton", driver.selectButton)
  }

  fun logOperatorController() {
    Logger.getInstance()
      .recordOutput("OperatorController/rightTriggerAxis", operator.rightTriggerAxis)
    Logger.getInstance()
      .recordOutput("OperatorController/leftTriggerAxis", operator.leftTriggerAxis)
    Logger.getInstance().recordOutput("OperatorController/startButton", operator.startButton)
    Logger.getInstance().recordOutput("OperatorController/selectButton", operator.selectButton)
    Logger.getInstance().recordOutput("OperatorController/aButton", operator.aButton)
    Logger.getInstance().recordOutput("OperatorController/bButton", operator.bButton)
    Logger.getInstance().recordOutput("OperatorController/xButton", operator.xButton)
    Logger.getInstance().recordOutput("OperatorController/yButton", operator.yButton)
    Logger.getInstance().recordOutput("OperatorController/dPadDown", operator.dPadDown)
    Logger.getInstance().recordOutput("OperatorController/dPadRight", operator.dPadRight)
    Logger.getInstance().recordOutput("OperatorController/dPadUp", operator.dPadUp)
    Logger.getInstance().recordOutput("OperatorController/dPadLeft", operator.dPadLeft)
    Logger.getInstance()
      .recordOutput("OperatorController/rightShoulderButton", operator.rightShoulderButton)
    Logger.getInstance()
      .recordOutput("OperatorController/leftShoulderButton", operator.leftShoulderButton)
  }
}
