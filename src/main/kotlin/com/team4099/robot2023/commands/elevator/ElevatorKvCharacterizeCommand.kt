package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

/**
 * This command is used to experimentally determine the kS value for the elevator It prints the
 * first voltage level that causes the elevator to move
 *
 * @property hasMoved used as end condition
 * @property appliedVolts the current amount of volts being applied to the motors
 * @property step the increase in volts per iteration
 */
class ElevatorKvCharacterizeCommand(val superstructure: Superstructure) : CommandBase() {
  init {
    addRequirements(superstructure)
  }

  var appliedVolts = 0.0.volts
  var sim_step = 0.001.volts
  var real_step = 0.01.volts

  override fun initialize() {
    appliedVolts = 0.0.volts
    superstructure.currentRequest = Request.SuperstructureRequest.Tuning()
  }

  override fun execute() {
    Logger.getInstance().recordOutput("Elevator/appliedVolts", appliedVolts.inVolts)
    superstructure.elevatorSetVoltage(appliedVolts)

    if (RobotBase.isReal()) {
      appliedVolts += real_step
    } else {
      appliedVolts += sim_step
    }
  }

  override fun isFinished(): Boolean {
    return false
  }
}
