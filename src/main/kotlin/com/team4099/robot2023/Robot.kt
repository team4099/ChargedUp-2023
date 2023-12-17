package com.team4099.robot2023

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.Alert.AlertType
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.NTSafePublisher
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.ejml.EjmlVersion.BUILD_DATE
import org.ejml.EjmlVersion.DIRTY
import org.ejml.EjmlVersion.GIT_BRANCH
import org.ejml.EjmlVersion.GIT_SHA
import org.ejml.EjmlVersion.MAVEN_NAME
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team4099.lib.units.base.inMilliseconds
import java.nio.file.Files
import java.nio.file.Paths

object Robot : LoggedRobot() {

  val logFolderAlert =
    Alert("Log folder path does not exist. Data will NOT be logged.", AlertType.ERROR)
  val logReceiverQueueAlert =
    Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR)
  val logOpenFileAlert = Alert("Failed to open log file. Data will NOT be logged", AlertType.ERROR)
  val logWriteAlert =
    Alert("Failed write to the log file. Data will NOT be logged", AlertType.ERROR)
  val logSimulationAlert = Alert("Running in simulation", AlertType.INFO)
  val logTuningModeEnabled =
    Alert("Tuning Mode Enabled. Expect loop times to be greater", AlertType.WARNING)
  /*
  val port0 = AnalogInput(0)
  val port1 = AnalogInput(1)
  val port2 = AnalogInput(2)
  val port3 = AnalogInput(3)

   */

  override fun robotInit() {
    // running replays as fast as possible when replaying. (play in real time when robot is real or
    // sim)
    setUseTiming(
      RobotBase.isReal() || Constants.Universal.SIM_MODE != Constants.Tuning.SimType.REPLAY
    )

    // Tuning mode alert check
    logTuningModeEnabled.set(Constants.Tuning.TUNING_MODE)

    // metadata value (not timed -- just metadata for given log file)
    Logger.recordMetadata(
      "Robot", if (RobotBase.isReal()) "REAL" else Constants.Universal.SIM_MODE.name
    )
    Logger.recordMetadata("Tuning Mode Enabled", Constants.Tuning.TUNING_MODE.toString())
    Logger.recordMetadata("ProjectName", MAVEN_NAME)
    Logger.recordMetadata("BuildDate", BUILD_DATE)
    Logger.recordMetadata("GitSHA", GIT_SHA)
    Logger.recordMetadata("GitBranch", GIT_BRANCH)
    when (DIRTY) {
      0 -> Logger.recordMetadata("GitDirty", "All changes committed")
      1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes")
      else -> Logger.recordMetadata("GitDirty", "Unknown")
    }

    if (RobotBase.isReal()) {
      // check if folder path exists
      if (Files.exists(Paths.get(Constants.Universal.LOG_FOLDER))) {
        // log to USB stick and network for real time data viewing on AdvantageScope
        Logger.addDataReceiver(WPILOGWriter(Constants.Universal.LOG_FOLDER))
      } else {
        logFolderAlert.set(true)
      }

      Logger.addDataReceiver(NTSafePublisher())
      LoggedPowerDistribution.getInstance(
        Constants.Universal.POWER_DISTRIBUTION_HUB_ID, PowerDistribution.ModuleType.kRev
      )
    } else {
      when (Constants.Tuning.SimType.SIM) {
        Constants.Tuning.SimType.SIM -> {
          Logger.addDataReceiver(NTSafePublisher())
          logSimulationAlert.set(true)
          DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        }
        Constants.Tuning.SimType.REPLAY -> {
          // if in replay mode get file path from command line and read log file
          val path = LogFileUtil.findReplayLog()
          Logger.setReplaySource(WPILOGReader(path))
          Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
        }
      }

      // initialize mech2d stuff
    }

    Logger.start() // no more configuration allowed

    LiveWindow.disableAllTelemetry()

    // init robot container too
    RobotContainer
    AutonomousSelector
    PathStore
    RobotContainer.mapDefaultCommands()

    // Set the scheduler to log events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", true)
    }

    CommandScheduler.getInstance().onCommandFinish { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }

    CommandScheduler.getInstance().onCommandInterrupt { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }
  }

  override fun autonomousInit() {
    RobotContainer.zeroSensors()
    FMSData.allianceColor = DriverStation.getAlliance().get()
    RobotContainer.setDriveBrakeMode()
    RobotContainer.setSteeringBrakeMode()
    RobotContainer.getAutonomousCommand().schedule()
  }

  override fun disabledInit() {
    RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setSteeringCoastMode()
    RobotContainer.setDriveBrakeMode()
    // autonomousCommand.cancel()
  }

  override fun robotPeriodic() {
    //    RobotContainer.measurementsWithTimestamps.forEach {
    // RobotContainer.addVisionMeasurement(it) }

    var startTime = Clock.realTimestamp

    // begin scheduling all commands
    CommandScheduler.getInstance().run()

    // checking for logging errors
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault())

    // motor checker stuff
    val motorCheckerStartTime = Clock.realTimestamp
    MotorChecker.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/MotorCheckerLoopTimeMS",
      (Clock.realTimestamp - motorCheckerStartTime).inMilliseconds
    )

    Logger.recordOutput(
      "LoggedRobot/RemainingRamMB", Runtime.getRuntime().freeMemory() / 1024 / 1024
    )

    Logger.recordOutput("LoggedRobot/totalMS", (Clock.realTimestamp - startTime).inMilliseconds)

    /*
    Logger.recordOutput("LoggedRobot/port0", port0.voltage)
    Logger.recordOutput("LoggedRobot/port1", port1.voltage)
    Logger.recordOutput("LoggedRobot/port2", port2.voltage)
    Logger.recordOutput("LoggedRobot/port3", port3.voltage)

     */
  }

  override fun teleopInit() {
    FMSData.allianceColor = DriverStation.getAlliance().get()
    RobotContainer.mapTeleopControls()
    RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setDriveBrakeMode()
    RobotContainer.setSteeringBrakeMode()
    if (Constants.Tuning.TUNING_MODE) {
      RobotContainer.mapTunableCommands()
    }
  }

  override fun testInit() {
    RobotContainer.mapTestControls()
    RobotContainer.getAutonomousCommand().cancel()
  }
}
