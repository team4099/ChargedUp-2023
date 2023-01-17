package com.team4099.robot2023.commands.led

import com.team4099.robot2023.Robot
import com.team4099.robot2023.config.constants.LedConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.led.Led
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

class LedCommand(val led: Led, val drivetrain: Drivetrain) : CommandBase() {

  val gyroAngle: Angle
    get() {
      return when (drivetrain.closestAlignmentAngle.absoluteValue.inDegrees % 180.0) {
        0.0 -> drivetrain.gyroInputs.gyroPitch
        90.0 -> drivetrain.gyroInputs.gyroRoll
        else -> 0.0.degrees
      }
    }

  init {
    addRequirements(led, drivetrain)
  }

  override fun execute() {

    // leds signify which direciton, forwards/backwards(field orriented) the robot should move to be
    // leveled
    if (gyroAngle.absoluteValue > 2.5.degrees) {
      if (drivetrain.closestAlignmentAngle.sign == -1.0) {
        led.state = LedConstants.LEDMode.BACKWARD_TO_LEVEL
      } else {
        led.state = LedConstants.LEDMode.FORWARD_TO_LEVEL
      }
    } else {
      led.state =
        when {
          // TODO finish once other subsystems are merged
          false -> LedConstants.LEDMode.OUTTAKE
          false -> LedConstants.LEDMode.ITEM
          Robot.isAutonomous -> LedConstants.LEDMode.AUTO
          Robot.isTeleop -> LedConstants.LEDMode.TELEOP
          Robot.isDisabled -> LedConstants.LEDMode.DISABLED
          else -> LedConstants.LEDMode.IDLE
        }
    }
  }
}
