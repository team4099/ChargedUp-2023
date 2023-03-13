package com.team4099.robot2023.auto

import com.team4099.robot2023.auto.mode.ConeCubeAuto
import com.team4099.robot2023.auto.mode.ConeCubeBumpAuto
import com.team4099.robot2023.auto.mode.ConeMobilityAuto
import com.team4099.robot2023.auto.mode.PreloadOpenLoopChargeStationBalance
import com.team4099.robot2023.auto.mode.TestAutoPath
import com.team4099.robot2023.commands.elevator.ElevatorKsCharacterizeCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

object AutonomousSelector {
  //  private var orientationChooser: SendableChooser<Angle> = SendableChooser()
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
    LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var secondaryWaitInAuto: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")
    //    orientationChooser.setDefaultOption("Forward", 0.degrees)
    //    orientationChooser.addOption("Backwards", 180.degrees)
    //    orientationChooser.addOption("Left", 90.degrees)
    //    orientationChooser.addOption("Right", 270.degrees)
    //    autoTab.add("Starting Orientation", orientationChooser)

    autonomousModeChooser.addOption("Test", AutonomousMode.TEST_AUTO_PATH)
    autonomousModeChooser.addOption("Characterize Elevator", AutonomousMode.ELEVATOR_CHARACTERIZE)
    autonomousModeChooser.addOption("1 Cone + 1 Cube Auto", AutonomousMode.CO_CU_AUTO)
    autonomousModeChooser.addOption(
      "1 Cone + 1 Cube Auto, Cable Carrier Side", AutonomousMode.CO_CU_BUMP_AUTO
    )
    autonomousModeChooser.addOption("1 Cone + Mobility", AutonomousMode.CONE_MOBILITY_AUTO)
    autonomousModeChooser.addOption("1 Cone + Open Loop Charge Station", AutonomousMode.CONE_MOBILITY_AUTO)

    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(5, 2).withPosition(3, 0)
    waitBeforeCommandSlider =
      autoTab
        .add("Wait Time before Running Auto", 0)
        .withSize(3, 2)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
    secondaryWaitInAuto =
      autoTab
        .add("Secondary Wait Time During Auto Path", 0)
        .withSize(3, 2)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  val secondaryWaitTime: Time
    get() = secondaryWaitInAuto.getDouble(0.0).seconds

  fun getCommand(drivetrain: Drivetrain, superstructure: Superstructure): CommandBase {

    val mode = autonomousModeChooser.get()
    //    println("${waitTime().inSeconds} wait command")
    when (mode) {
      AutonomousMode.TEST_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds).andThen(TestAutoPath(drivetrain))
      AutonomousMode.ELEVATOR_CHARACTERIZE -> return ElevatorKsCharacterizeCommand(superstructure)
      AutonomousMode.CO_CU_AUTO ->
        return WaitCommand(waitTime.inSeconds).andThen(ConeCubeAuto(drivetrain, superstructure))
      AutonomousMode.CO_CU_BUMP_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeBumpAuto(drivetrain, superstructure))
      AutonomousMode.CONE_MOBILITY_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeMobilityAuto(drivetrain, superstructure))
      AutonomousMode.PRELOAD_SCORE_OPEN_LOOP_CHARGE_STATION_SCORE ->
        return WaitCommand(waitTime.inSeconds).andThen(PreloadOpenLoopChargeStationBalance(drivetrain, superstructure))
      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  private enum class AutonomousMode {
    TEST_AUTO_PATH,
    ELEVATOR_CHARACTERIZE,
    CO_CU_AUTO,
    CO_CU_BUMP_AUTO,
    CONE_MOBILITY_AUTO,
    PRELOAD_SCORE_OPEN_LOOP_CHARGE_STATION_SCORE
  }
}
