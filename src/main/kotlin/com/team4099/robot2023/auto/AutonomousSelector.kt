package com.team4099.robot2023.auto

import com.team4099.robot2023.auto.mode.ConeCubeAuto
import com.team4099.robot2023.auto.mode.ConeCubeBumpAuto
import com.team4099.robot2023.auto.mode.ConeCubeHoldAuto
import com.team4099.robot2023.auto.mode.ConeCubeHoldBumpAuto
import com.team4099.robot2023.auto.mode.ConeCubeHoldOverChargeStationAuto
import com.team4099.robot2023.auto.mode.ConeCubeLaunchOverChargeStationAuto
import com.team4099.robot2023.auto.mode.ConeCubeLowOverChargeStationAuto
import com.team4099.robot2023.auto.mode.ConeCubeMobilityAuto
import com.team4099.robot2023.auto.mode.ConeCubeOverChargeStationAuto
import com.team4099.robot2023.auto.mode.ConeMobilityAuto
import com.team4099.robot2023.auto.mode.PreloadOpenLoopChargeStationBalance
import com.team4099.robot2023.auto.mode.ScorePreloadCone
import com.team4099.robot2023.auto.mode.TestAutoPath
import com.team4099.robot2023.commands.drivetrain.PositionAutoLevel
import com.team4099.robot2023.commands.elevator.ElevatorKsCharacterizeCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.Logger
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
  private var autoEngageWidgit: GenericEntry
  var hasCube: GenericEntry
  var hasCone: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")
    //    orientationChooser.setDefaultOption("Forward", 0.degrees)
    //    orientationChooser.addOption("Backwards", 180.degrees)
    //    orientationChooser.addOption("Left", 90.degrees)
    //    orientationChooser.addOption("Right", 270.degrees)
    //    autoTab.add("Starting Orientation", orientationChooser)

    // autonomousModeChooser.addOption("Test", AutonomousMode.TEST_AUTO_PATH)
    // autonomousModeChooser.addOption("Characterize Elevator",
    // AutonomousMode.ELEVATOR_CHARACTERIZE)

    autonomousModeChooser.addOption("1 Cone + 1 Cube Auto", AutonomousMode.CO_CU_AUTO)
    autonomousModeChooser.addOption(
      "1 Cone + 1 Cube Mobility Auto", AutonomousMode.CO_CU_MOBILITY_AUTO
    )

    // autonomousModeChooser.addOption("1 Cone + 1 Cube Auto, 254 version",
    // AutonomousMode.CO_CU_AUTO_NO_SPIN)

    autonomousModeChooser.addOption(
      "1 Cone + 1 Cube Auto, Cable Carrier Side", AutonomousMode.CO_CU_BUMP_AUTO
    )

    autonomousModeChooser.addOption(
      "1 Cone + 1 Cube Auto, Goes Over Charge Station", AutonomousMode.CO_CU_MIDDLE_AUTO
    )

    autonomousModeChooser.addOption(
      "1 Cone + 1 Low Cube Auto, Goes Over Charge Station", AutonomousMode.CO_CU_MIDDLE_LOW_AUTO
    )

    autonomousModeChooser.addOption("1 Cone + Hold Cube", AutonomousMode.CO_CU_HOLD_AUTO)
    autonomousModeChooser.addOption(
      "1 Cone + Hold Cube, Cable Carrier Side", AutonomousMode.CO_CU_BUMP_HOLD_AUTO
    )
    autonomousModeChooser.addOption(
      "1 Cone + Hold Cube, Goes Over Charge Station", AutonomousMode.CO_CU_HOLD_MIDDLE_AUTO
    )

    autonomousModeChooser.addOption(
      "1 Cone + Launch Cube, Goes Over Charge Station", AutonomousMode.CO_CU_LAUNCH_MIDDLE_AUTO
    )

    autonomousModeChooser.addOption("1 Cone + Mobility", AutonomousMode.CONE_MOBILITY_AUTO)

    // autonomousModeChooser.addOption("1 Cone + Open Loop Charge Station",
    // AutonomousMode.CONE_MOBILITY_AUTO)

    autonomousModeChooser.addOption("Score Preload Cone", AutonomousMode.PRELOAD_SCORE_AUTO)

    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    autoEngageWidgit =
      autoTab
        .add("Auto Engage", false)
        .withSize(2, 2)
        .withPosition(6, 0)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .entry

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

    hasCube =
      autoTab
        .add("Has Cube", false)
        .withPosition(7, 2)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .entry
    hasCone =
      autoTab
        .add("Has Cone", false)
        .withPosition(6, 2)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  val secondaryWaitTime: Time
    get() = secondaryWaitInAuto.getDouble(0.0).seconds

  val attemptEngage: Boolean
    get() = autoEngageWidgit.getBoolean(false)

  fun getCommand(drivetrain: Drivetrain, superstructure: Superstructure): CommandBase {

    var engageCommand: CommandBase = InstantCommand()
    if (attemptEngage) {
      engageCommand = PositionAutoLevel(drivetrain)
    } else {
      Logger.getInstance().recordOutput("AttemptAutoEngage", false)
    }

    val mode = autonomousModeChooser.get()
    //    println("${waitTime().inSeconds} wait command")
    when (mode) {
      AutonomousMode.TEST_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds).andThen(TestAutoPath(drivetrain))
      AutonomousMode.ELEVATOR_CHARACTERIZE -> return ElevatorKsCharacterizeCommand(superstructure)
      AutonomousMode.CO_CU_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_MOBILITY_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeMobilityAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_BUMP_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeBumpAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_MIDDLE_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeOverChargeStationAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_MIDDLE_LOW_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeLowOverChargeStationAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_HOLD_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeHoldAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_BUMP_HOLD_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeHoldBumpAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_HOLD_MIDDLE_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeHoldOverChargeStationAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.CO_CU_LAUNCH_MIDDLE_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeCubeLaunchOverChargeStationAuto(drivetrain, superstructure))
      AutonomousMode.CONE_MOBILITY_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ConeMobilityAuto(drivetrain, superstructure))
          .andThen(engageCommand)
      AutonomousMode.PRELOAD_SCORE_OPEN_LOOP_CHARGE_STATION_SCORE ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(PreloadOpenLoopChargeStationBalance(drivetrain, superstructure))
      AutonomousMode.PRELOAD_SCORE_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen(ScorePreloadCone(drivetrain, superstructure))
          .andThen(engageCommand)
      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  private enum class AutonomousMode {
    TEST_AUTO_PATH,
    ELEVATOR_CHARACTERIZE,
    CO_CU_AUTO,
    CO_CU_MOBILITY_AUTO,
    CO_CU_BUMP_AUTO,
    CO_CU_MIDDLE_AUTO,
    CO_CU_MIDDLE_LOW_AUTO,
    CO_CU_HOLD_AUTO,
    CO_CU_BUMP_HOLD_AUTO,
    CO_CU_HOLD_MIDDLE_AUTO,
    CO_CU_LAUNCH_MIDDLE_AUTO,
    CONE_MOBILITY_AUTO,
    PRELOAD_SCORE_OPEN_LOOP_CHARGE_STATION_SCORE,
    PRELOAD_SCORE_AUTO
  }
}
