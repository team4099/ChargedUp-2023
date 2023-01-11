package com.team4099.robot2023.commands.led

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
    /* led.state =
        when (Robot.isDisabled) {
          LedConstants.LEDMode.DISABLED
        }
    */

    if (kotlin.math.abs(drivetrain.gyroInputs.gyroPitch.inDegrees) > 2.5){
      if(drivetrain.gyroInputs.gyroPitch.inDegrees > 2.5){

      }
      else{

      }
    }
    else{

    }
  }
}
