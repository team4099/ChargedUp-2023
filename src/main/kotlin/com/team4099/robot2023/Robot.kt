package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.MechanismSimConstants
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.Alert.AlertType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
    setUseTiming(
      RobotBase.isReal() || Constants.Universal.SIM_MODE != Constants.Tuning.SimType.REPLAY
    )

    // metadata value (not timed -- just metadata for given log file)
    logger.recordMetadata(
      "Robot", if (RobotBase.isReal()) "REAL" else Constants.Universal.SIM_MODE.name
    )
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

    if (RobotBase.isReal()) {
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
    } else {
      when (Constants.Universal.SIM_MODE) {
        Constants.Tuning.SimType.SIM -> {
          logger.addDataReceiver(NT4Publisher())
          logSimulationAlert.set(true)
        }
        Constants.Tuning.SimType.REPLAY -> {
          // if in replay mode get file path from command line and read log file
          val path = LogFileUtil.findReplayLog()
          logger.setReplaySource(WPILOGReader(path))
          logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")))
        }
      }

      // initialize mech2d stuff
    }

    logger.start() // no more configuration allowed

    LiveWindow.disableAllTelemetry()

    // init robot container too
    RobotContainer
    AutonomousSelector
    PathStore
    // RobotContainer.zeroSensors() UNCOMMENT THIS PLS
    RobotContainer.mapDefaultCommands()
  }

  override fun autonomousInit() {
    // autonomousCommand.schedule()
    RobotContainer.setDriveBrakeMode()
    //    RobotContainer.zeroSteering()
    RobotContainer.getAutonomousCommand().schedule()
  }

  override fun disabledInit() {
    RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setDriveBrakeMode()
    // autonomousCommand.cancel()
  }

  override fun robotPeriodic() {
    // begin scheduling all commands
    CommandScheduler.getInstance().run()

    // checking for logging errors
    logReceiverQueueAlert.set(Logger.getInstance().receiverQueueFault)

    // Set the scheduler to log events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize { command: Command ->
      Logger.getInstance().recordOutput("/ActiveCommands/${command.name}", true)
    }

    CommandScheduler.getInstance().onCommandFinish { command: Command ->
      Logger.getInstance().recordOutput("/ActiveCommands/${command.name}", false)
    }

    CommandScheduler.getInstance().onCommandInterrupt { command: Command ->
      Logger.getInstance().recordOutput("/ActiveCommands/${command.name}", false)
    }

    if (!RobotBase.isReal()) {
      SmartDashboard.putData("Arm Sim", MechanismSimConstants.m_mech2d)
    }
  }

  override fun teleopInit() {
    RobotContainer.mapTeleopControls()
    // RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.setDriveBrakeMode() // change to coast
    //    RobotContainer.zeroSteering()
    // autonomousCommand.cancel()
    if (Constants.Tuning.TUNING_MODE) {
      RobotContainer.mapTunableCommands()
    }
  }

  override fun testInit() {
    RobotContainer.mapTestControls()
    RobotContainer.getAutonomousCommand().cancel()
  }
}
