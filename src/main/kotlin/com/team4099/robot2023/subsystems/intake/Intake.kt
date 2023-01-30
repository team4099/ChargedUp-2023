package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class Intake(val io: IntakeIO) : SubsystemBase() {

  val inputs = IntakeIO.IntakeIOInputs()

  var armFeedforward =
    ArmFeedforward(
      IntakeConstants.NEO_ARM_KS,
      IntakeConstants.ARM_KG,
      IntakeConstants.ARM_KV,
      IntakeConstants.ARM_KA
    )

  private val kP =
    LoggedTunableValue("Intake/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "Intake/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Intake/kd", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private val rollerKV =
    LoggedTunableValue(
      "Intake/rollerKV",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= IntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= IntakeConstants.ARM_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  val rollerState: IntakeConstants.rollerStates
    get() {
      for (state in IntakeConstants.rollerStates.values()) {
        if ((state.velocity - inputs.armVelocity).absoluteValue <=
          IntakeConstants.ROLLERR_TOLERANCE
        ) {
          return state
        }
      }
      return IntakeConstants.rollerStates.DUMMY
    }

  val armState: IntakeConstants.armStates
    get() {
      for (state in IntakeConstants.armStates.values()) {
        if ((state.position - inputs.armPosition).absoluteValue <= IntakeConstants.ARM_TOLERANCE) {
          return state
        }
      }
      return IntakeConstants.armStates.DUMMY
    }

  var armConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      IntakeConstants.MAX_ARM_VELOCITY, IntakeConstants.MAX_ARM_ACCELERATION
    )

  var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  var positionToHold = inputs.armPosition

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(IntakeConstants.NEO_KP)
      kI.initDefault(IntakeConstants.NEO_KI)
      kD.initDefault(IntakeConstants.NEO_KD)
      rollerKV.initDefault(IntakeConstants.NEO_ROLLER_KV)

      armFeedforward =
        ArmFeedforward(
          IntakeConstants.NEO_ARM_KS,
          IntakeConstants.ARM_KG,
          IntakeConstants.ARM_KV,
          IntakeConstants.ARM_KA
        )
    } else {
      kP.initDefault(IntakeConstants.SIM_KP)
      kI.initDefault(IntakeConstants.SIM_KI)
      kD.initDefault(IntakeConstants.SIM_KD)
      rollerKV.initDefault(IntakeConstants.SIM_ROLLER_KV)

      var armFeedforward =
        ArmFeedforward(
          IntakeConstants.SIM_ARM_KS,
          IntakeConstants.ARM_KG,
          IntakeConstants.ARM_KV,
          IntakeConstants.ARM_KA
        )
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Intake", inputs)
  }

  /** @param rpm Represents the angular velocity of the rollers */
  fun setRollerPower(rpm: AngularVelocity) {
    val voltage = (rpm * rollerKV.get())

    io.setRollerPower(voltage)
    Logger.getInstance().recordOutput("Intake/rollerTargetSpeedRPM", rpm.inRotationsPerMinute)
    Logger.getInstance().recordOutput("Intake/RollerTargetVotlage", voltage.inVolts)
  }

  fun setRollerBrakeMode(brake: Boolean) {
    io.setRollerBrakeMode(brake)
    Logger.getInstance().recordOutput("Intake/rollerBrakeModeEnabled", brake)
  }

  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
    Logger.getInstance().recordOutput("Intake/armBrakeModeEnabled", brake)
  }

  fun holdArmPosition(): Command {
    positionToHold = inputs.armPosition
    return run {
      io.setArmPosition(positionToHold, armFeedforward.calculate(0.degrees, 0.degrees.perSecond))

      Logger.getInstance().recordOutput("Intake/holdPosition", positionToHold.inDegrees)
      Logger.getInstance().recordOutput("Intake/ActiveCommands/holdArmCommand", true)
    }
      .finallyDo {
        Logger.getInstance().recordOutput("Intake/ActiveCommands/holdArmCommand", false)
      }
  }

  fun setArmPosition(setpoint: TrapezoidProfile.State<Radian>) {
    val armAngularAcceleration =
      (setpoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setpoint

    val feedforward =
      armFeedforward.calculate(setpoint.position, setpoint.velocity, armAngularAcceleration)

    if ((forwardLimitReached && setpoint.velocity > 0.degrees.perSecond) ||
      (reverseLimitReached && setpoint.velocity < 0.degrees.perSecond)
    ) {
      io.setArmVoltage(armFeedforward.calculate(0.degrees, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("Intake/intakeArmTargetPosition", setpoint.position.inDegrees)
  }

  /**
   * Command for creating a profile to a position and following that profile until reached
   *
   * @param position The angle the arm should go to
   */
  fun rotateArmPosition(position: Angle): Command {
    // Generate a trapezoidal profile from the current position to the setpoint
    // with set constraints
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(position, 0.degrees.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )

    // Initializing a variable of start time which represents the start of elapsed
    // time of the profile
    var startTime = Clock.fpgaTime

    // Creates and returns a command that can be run continuously until the profile finishes
    return run {
      // Regenerates profile state for that loop cycle and sets to that position
      setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
      Logger.getInstance()
        .recordOutput(
          "Intake/isAtSetpoint",
          (position - inputs.armPosition).absoluteValue <= IntakeConstants.ARM_TOLERANCE
        )
    }
      .beforeStarting(
        {
          Logger.getInstance().recordOutput("Intake/ActiveCommands/setArmPositionCommand", true)
          // Resets the initial time since the time at the start of method is of robot init
          startTime = Clock.fpgaTime
          Logger.getInstance().recordOutput("Intake/isAtSetpoint", false)
          // Regenerates profile with the new position passed in
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(position, 0.degrees.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
        },
        this
      )
      .until {
        // Run the lambda until the predicted finishing time of the profile elapses
        armProfile.isFinished(Clock.fpgaTime - startTime)
      }
      .finallyDo {
        positionToHold = position
        Logger.getInstance().recordOutput("Intake/ActiveCommands/setArmPositionCommand", false)
      }
      .handleInterrupt({ positionToHold = inputs.armPosition })
  }
}
