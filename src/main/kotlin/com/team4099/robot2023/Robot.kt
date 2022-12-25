package com.team4099.robot2023

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.Alert.AlertType
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
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

  override fun robotInit() {
    val logger = Logger.getInstance()
    // running replays as fast as possible when replaying. (play in real time when robot is real or
    // sim)
    setUseTiming(Constants.Universal.ROBOT_MODE != Constants.Tuning.RobotType.REPLAY)

    // metadata value (not timed -- just metadata for given log file)
    logger.recordMetadata("Robot", Constants.Universal.ROBOT_MODE.toString())
    logger.recordMetadata("Tuning Mode Enabled", Constants.Tuning.TUNING_MODE.toString())
    logger.recordMetadata("ProjectName", MAVEN_NAME)
    logger.recordMetadata("BuildDate", BUILD_DATE)
    logger.recordMetadata("GitSHA", GIT_SHA)
    logger.recordMetadata("GitBranch", GIT_BRANCH)
    when (DIRTY) {
      0 -> logger.recordMetadata("GitDirty", "All changes committed")
      1 -> logger.recordMetadata("GitDirty", "Uncommitted changes")
      else -> logger.recordMetadata("GitDirty", "Unknown")
    }

    when (Constants.Universal.ROBOT_MODE) {
      Constants.Tuning.RobotType.REAL -> {
        // check if folder path exists
        if (Files.exists(Paths.get(Constants.Universal.LOG_FOLDER))) {
          // log to USB stick and network for real time data viewing on AdvantageScope
          logger.addDataReceiver(WPILOGWriter(Constants.Universal.LOG_FOLDER))
        } else {
          logFolderAlert.set(true)
        }

        logger.addDataReceiver(NT4Publisher())
        LoggedPowerDistribution.getInstance(
          Constants.Universal.POWER_DISTRIBUTION_HUB_ID, PowerDistribution.ModuleType.kRev
        )
      }
      Constants.Tuning.RobotType.SIM -> {
        logger.addDataReceiver(NT4Publisher())
        logSimulationAlert.set(true)
      }
      Constants.Tuning.RobotType.REPLAY -> {
        // if in replay mode get file path from command line and read log file
        val path = LogFileUtil.findReplayLog()
        logger.setReplaySource(WPILOGReader(path))
        logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")))
      }
    }

    logger.start() // no more configuration allowed

    LiveWindow.disableAllTelemetry()

    // init robot container too
    RobotContainer
    // RobotContainer.zeroSensors() UNCOMMENT THIS PLS
    RobotContainer.mapDefaultCommands()
  }

  override fun autonomousInit() {
    // autonomousCommand.schedule()
    RobotContainer.setDriveBrakeMode()
    //    RobotContainer.zeroSteering()
    //    RobotContainer.getAutonomousCommand().schedule()
  }

  override fun disabledInit() {
    // RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setDriveBrakeMode()
    // autonomousCommand.cancel()
  }

  override fun robotPeriodic() {
    // begin scheduling all commands
    CommandScheduler.getInstance().run()

    // checking for logging errors
    logReceiverQueueAlert.set(Logger.getInstance().receiverQueueFault)

    // logging all commands
    Logger.getInstance()
      .recordOutput(
        "ActiveCommands/Scheduler",
        NetworkTableInstance.getDefault()
          .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
          .getStringArray(emptyArray())
      )
  }

  override fun teleopInit() {
    RobotContainer.mapTeleopControls()
    // RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setDriveBrakeMode() // change to coast
    //    RobotContainer.zeroSteering()
    // autonomousCommand.cancel()
  }

  override fun testInit() {
    RobotContainer.mapTestControls()
    // RobotContainer.getAutonomousCommand().cancel()
  }
}
