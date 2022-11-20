package com.team4099.robot2022

import com.team4099.robot2022.commands.elevator.ElevatorIdleCommand
import com.team4099.robot2022.commands.elevator.OpenLoopExtendCommand
import com.team4099.robot2022.commands.elevator.OpenLoopRetractCommand
import com.team4099.robot2022.config.ControlBoard
import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.subsytems.elevator.Elevator
import com.team4099.robot2022.subsytems.elevator.ElevatorIO
import com.team4099.robot2022.subsytems.elevator.ElevatorIOSim

object RobotContainer {
  private val elevator: Elevator

  init {
    if (Constants.Universal.ROBOT_MODE == Constants.Tuning.RobotType.REAL) {
      elevator = Elevator(object : ElevatorIO {})
    } else {
      elevator = Elevator(ElevatorIOSim)
    }
  }

  fun mapDefaultCommands() {
    elevator.defaultCommand = ElevatorIdleCommand(elevator)
  }

  fun mapTeleopControls() {
    ControlBoard.openLoopExtend.whileActiveContinuous(OpenLoopExtendCommand(elevator))
    ControlBoard.openLoopRetract.whileActiveContinuous(OpenLoopRetractCommand(elevator))
  }
}
