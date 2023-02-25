package com.team4099.robot2023.commands.elevator

import com.team4099.robot2023.commands.SysIdCommand
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import java.util.function.Consumer

class ElevatorSysIDCommand(private val superstructure: Superstructure) : SequentialCommandGroup() {

  init {
    val elevatorSetter =
      Consumer<Double> { voltage: Double -> superstructure.elevatorSetVoltage(voltage.volts) }

    val elevatorGetter = {
      SysIdCommand.MechanismSysIdData(
        superstructure.elevatorInputs.elevatorPosition.inMeters /
          ElevatorConstants.SPOOL_RADIUS.inMeters,
        superstructure.elevatorInputs.elevatorVelocity.inMetersPerSecond /
          ElevatorConstants.SPOOL_RADIUS.inMeters
      )
    }

    addCommands(SysIdCommand(superstructure, elevatorSetter, elevatorGetter))
  }
}
