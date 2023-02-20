package com.team4099.robot2023.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import java.util.function.BiConsumer
import java.util.function.Consumer
import java.util.function.Supplier

class SysIdCommand : CommandBase {
  private val isDriveTrain: Boolean
  private lateinit var driveTrainSetter: BiConsumer<Double, Double>
  private lateinit var mechanismSetter: Consumer<Double>
  private lateinit var driveTrainGetter: Supplier<DriveTrainSysIdData>
  private lateinit var mechanismGetter: Supplier<MechanismSysIdData>
  private var startTime = 0.0
  private lateinit var data: String

  /** Creates a new SysIdCommand for a drive train. */
  constructor(
    subsystem: Subsystem,
    driveTrainSetter: BiConsumer<Double, Double>,
    driveTrainGetter: Supplier<DriveTrainSysIdData>
  ) {
    addRequirements(subsystem)
    isDriveTrain = true
    this.driveTrainSetter = driveTrainSetter
    this.driveTrainGetter = driveTrainGetter
  }

  /** Creates a new SysIdCommand for a generic mechanism. */
  constructor(
    subsystem: Subsystem,
    mechanismSetter: Consumer<Double>,
    mechanismGetter: Supplier<MechanismSysIdData>
  ) {
    addRequirements(subsystem)
    isDriveTrain = false
    this.mechanismSetter = mechanismSetter
    this.mechanismGetter = mechanismGetter
  }

  // Called when the command is initially scheduled.
  override fun initialize() {
    SmartDashboard.putBoolean("SysIdOverflow", false)
    SmartDashboard.putString("SysIdTelemetry", "")
    startTime = Timer.getFPGATimestamp()
    data = ""
  }

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    val timestamp = Timer.getFPGATimestamp()

    // Check if running the correct test
    val test = SmartDashboard.getString("SysIdTest", "Drivetrain")
    val correctTest: Boolean =
      if (isDriveTrain) {
        test == "Drivetrain" || test == "Drivetrain (Angular)"
      } else {
        test == "Arm" || test == "Elevator" || test == "Simple"
      }
    SmartDashboard.putBoolean("SysIdWrongMech", !correctTest)

    // Wrong test, prevent movement
    if (!correctTest) {
      if (isDriveTrain) {
        driveTrainSetter.accept(0.0, 0.0)
      } else {
        mechanismSetter.accept(0.0)
      }
      return
    }

    // Calculate voltage
    val testType = SmartDashboard.getString("SysIdTestType", "")
    val voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0)
    val rotate = SmartDashboard.getBoolean("SysIdRotate", false)
    val baseVoltage: Double =
      when (testType) {
        "Quasistatic" -> voltageCommand * (timestamp - startTime)
        "Dynamic" -> voltageCommand
        else -> 0.0
      }
    val primaryVoltage = baseVoltage * if (rotate) -1 else 1

    // Set output and get new data
    if (isDriveTrain) {
      driveTrainSetter.accept(primaryVoltage, baseVoltage)
      val subsystemData = driveTrainGetter.get()
      data += "$timestamp,"
      data += "$primaryVoltage,"
      data += "$baseVoltage,"
      data += (subsystemData.leftPosRad / (2 * Math.PI)).toString() + ","
      data += (subsystemData.rightPosRad / (2 * Math.PI)).toString() + ","
      data += (subsystemData.leftVelRadPerSec / (2 * Math.PI)).toString() + ","
      data += ((subsystemData.rightVelRadPerSec / (2 * Math.PI)).toString() + ",")
      data += "${subsystemData.gyroPosRad},"
      data += "${subsystemData.gyroVelRadPerSec},"
    } else {
      mechanismSetter.accept(primaryVoltage)
      val subsystemData = mechanismGetter.get()
      data += "$timestamp,"
      data += "$primaryVoltage,"
      data += (subsystemData.posRad / (2 * Math.PI)).toString() + ","
      data += (subsystemData.velRadPerSec / (2 * Math.PI)).toString() + ","
    }

    Logger.getInstance().recordOutput("ActiveCommands/SysIdCommand", true)
  }

  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean) {
    if (data.length > 0) {
      SmartDashboard.putString("SysIdTelemetry", data.substring(0, data.length - 1))
      println("Saved " + java.lang.Long.toString(Math.round(data.length / 1024.0)) + " KB of data.")
    } else {
      println("No data to save. Something's gone wrong here...")
    }
    if (isDriveTrain) {
      driveTrainSetter.accept(0.0, 0.0)
    } else {
      mechanismSetter.accept(0.0)
    }
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return false
  }

  /** SysId data for a drivetrain, returned by the subsystem. */
  /**
   * Creates a new DriveTrainSysIdData.
   *
   * @param leftPosRad Left position (radians)
   * @param rightPosRad Right position (radians)
   * @param leftVelRadPerSec Left velocity (radians per second)
   * @param rightVelRadPerSec Right velocity (radians per second)
   * @param gyroPosRad Gyro position (radians)
   * @param gyroVelRadPerSec Gyro position (radians per second)
   */
  class DriveTrainSysIdData(
    val leftPosRad: Double,
    val rightPosRad: Double,
    val leftVelRadPerSec: Double,
    val rightVelRadPerSec: Double,
    val gyroPosRad: Double,
    val gyroVelRadPerSec: Double
  )

  /** SysId data for a generic mechanism, returned by the subsystem. */
  class MechanismSysIdData
  /**
   * Creates a new MechanismSysIdData.
   *
   * @param posRad Position (radians)
   * @param velRadPerSec Velocity (radians per second)
   */
    (val posRad: Double, val velRadPerSec: Double)
}
