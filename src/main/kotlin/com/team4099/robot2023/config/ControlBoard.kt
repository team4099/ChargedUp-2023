package com.team4099.robot2023.config

import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad
import java.util.function.Consumer

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {
  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val rumbleConsumer =
    Consumer<Boolean> {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
    }

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis

  val slowMode: Boolean
    get() = driver.leftShoulderButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  val extendArm = Trigger { operator.aButton }

  val retractArm = Trigger { operator.bButton }

  // val setArmPositionToShelfIntake = Trigger { operator.xButton }

  // val toConeLevelOnePrep = Trigger {operator.leftShoulderButton && operator.aButton}

  val prepScore = Trigger { operator.leftTriggerAxis > 0.5 }

  val setArmCubeHybridPrep = Trigger { operator.leftShoulderButton && operator.aButton }
  val setArmCubeMidPrep = Trigger { operator.leftShoulderButton && operator.bButton }
  val setArmCubeHighPrep = Trigger { operator.leftShoulderButton && operator.yButton }

  val setArmConeHybridPrep = Trigger { operator.rightShoulderButton && operator.aButton }
  val setArmConeMidPrep = Trigger { operator.rightShoulderButton && operator.bButton }
  val setArmConeHighPrep = Trigger { operator.rightShoulderButton && operator.yButton }

  val goBackToIdle = Trigger { operator.selectButton && operator.startButton }

  val setArmDoubleSubCube = Trigger { operator.dPadLeft }
  val setArmDoubleSubCone = Trigger { operator.dPadRight }

  val doubleSubstationIntake = Trigger { driver.aButton }
  val singleSubstationIntake = Trigger { driver.bButton }
  val scoreOuttake = Trigger { driver.xButton }
  val groundIntakeCube = Trigger { driver.rightShoulderButton }

<<<<<<< Updated upstream
//  val increaseRollerVoltage = Trigger { operator.dPadUp }
//  val decreaseRollerVoltage = Trigger { operator.dPadDown }

  val groundIntakeCone = Trigger { driver.yButton }
  val autoScore = Trigger { driver.dPadDown}
//  val dpadDown = Trigger { driver.dPadDown }
=======
  //  val increaseRollerVoltage = Trigger { operator.dPadUp }
  //  val decreaseRollerVoltage = Trigger { operator.dPadDown }

  val groundIntakeCone = Trigger { driver.yButton }
  val autoScore = Trigger { driver.dPadDown }
  //  val dpadDown = Trigger { driver.dPadDown }
>>>>>>> Stashed changes

  val ejectGamePiece = Trigger { operator.dPadRight }

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
