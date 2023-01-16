package com.team4099.robot2023.commands.led

import com.team4099.robot2023.Robot
import com.team4099.robot2023.config.constants.LedConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.led.Led
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.inDegrees
import java.lang.Math.abs

class LedCommand(val led: Led, val drivetrain: Drivetrain) : CommandBase() {

  init {
    addRequirements(led, drivetrain)
  }

  override fun execute() {

    if (kotlin.math.abs(drivetrain.gyroInputs.gyroPitch.inDegrees) > 2.5) {
      if (drivetrain.gyroInputs.gyroPitch.inDegrees > 2.5) {
        led.state = LedConstants.LEDMode.POS_LEVELED
      } else {
        led.state = LedConstants.LEDMode.NEG_LEVELED
      }
    } else {
      led.state =
        when {
          // elevator is equal to height1, height2, or height3
          false -> LedConstants.LEDMode.OUTTAKE
          // if holding item
          false -> LedConstants.LEDMode.ITEM
          Robot.isAutonomous -> LedConstants.LEDMode.AUTO
          Robot.isTeleop -> LedConstants.LEDMode.TELEOP
          Robot.isDisabled -> LedConstants.LEDMode.DISABLED
          else -> LedConstants.LEDMode.IDLE
        }
    }
  }
}
