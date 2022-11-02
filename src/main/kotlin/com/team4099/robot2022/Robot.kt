package com.team4099.robot2022

import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.util.Alert
import com.team4099.robot2022.util.Alert.AlertType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggedNetworkTables
import org.littletonrobotics.junction.inputs.LoggedSystemStats
import org.littletonrobotics.junction.io.ByteLogReceiver
import org.littletonrobotics.junction.io.ByteLogReplay
import org.littletonrobotics.junction.io.LogSocketServer
import java.nio.file.Files
import java.nio.file.Paths

object Robot : LoggedRobot() {

  private lateinit var logReceiver: ByteLogReceiver

  val logFolderAlert =
    Alert("Log folder path does not exist. Data will NOT be logged.", AlertType.ERROR)
  val logReceiverQueueAlert =
    Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR)
  val logOpenFileAlert = Alert("Failed to open log file. Data will NOT be logged", AlertType.ERROR)
  val logWriteAlert =
    Alert("Failed write to the log file. Data will NOT be logged", AlertType.ERROR)

  override fun robotInit() {
    val logger = Logger.getInstance()

    // running replays as fast as possible when replaying. (play in real time when robot is real or
    // sim)
    setUseTiming(Constants.Universal.ROBOT_MODE != Constants.Tuning.RobotType.REPLAY)

    // log smart dashboard values (otherwise nothing is logged by default)
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard")

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
          logReceiver = ByteLogReceiver(Constants.Universal.LOG_FOLDER)
          logger.addDataReceiver(logReceiver)
        } else {
          logFolderAlert.set(true)
        }

        logger.addDataReceiver(LogSocketServer(5800))
        LoggedSystemStats.getInstance()
          .setPowerDistributionConfig(1, PowerDistribution.ModuleType.kRev)
      }
      Constants.Tuning.RobotType.SIM -> {
        logger.addDataReceiver(LogSocketServer(5800))
      }
      Constants.Tuning.RobotType.REPLAY -> {
        // if in replay mode get file path from command line and read log file
        val path = ByteLogReplay.promptForPath()
        logger.setReplaySource(ByteLogReplay(path))
        logger.addDataReceiver(ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")))
        logger.addDataReceiver(LogSocketServer(5800))
      }
    }

    logger.start() // no more configuration allowed

    LiveWindow.disableAllTelemetry()

    // init robot container too
  }

  override fun robotPeriodic() {
    // begin scheduling all commands
    CommandScheduler.getInstance().run()

    // checking for logging errors
    logReceiverQueueAlert.set(Logger.getInstance().receiverQueueFault)
    if (logReceiver != null) {
      logOpenFileAlert.set(logReceiver.openFault)
      logWriteAlert.set(logReceiver.writeFault)
    }
  }
}
