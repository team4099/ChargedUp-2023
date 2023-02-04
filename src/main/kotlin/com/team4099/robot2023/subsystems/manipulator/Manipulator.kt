package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchPerSecond
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import java.util.function.Supplier

class Manipulator(val io: ManipulatorIO) : SubsystemBase() {
  val inputs = ManipulatorIO.ManipulatorIOInputs()
  // placement feedforward
  val armFeedforward =
    SimpleMotorFeedforward(
      ManipulatorConstants.ARM_KS, ManipulatorConstants.ARM_KV, ManipulatorConstants.ARM_KA
    )
  private val kP =
    LoggedTunableValue("Manipulator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val kI =
    LoggedTunableValue(
      "Manipulator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Manipulator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts.perInchPerSecond })
    )

  // checks if motor current draw is greater than given threshold and if rollers are intaking
  // last condition prevnts current spikes caused by starting to run intake from triggering this
  var lastRollerRunTime = Clock.fpgaTime

  val rollerState: ManipulatorConstants.RollerStates
    get() = ManipulatorConstants.RollerStates.fromRollerVoltageToState(inputs.rollerAppliedVoltage)

  var lastRollerState = rollerState

  // Checks if motor current draw is greater than given threshold and if rollers are intaking
  // Last condition prevents current spikes caused by starting to run intake from triggering this
  val hasCube: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CUBE_CURRENT_THRESHOLD &&
        (
          rollerState == ManipulatorConstants.RollerStates.CUBE_IN ||
            rollerState == ManipulatorConstants.RollerStates.CUBE_IDLE
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  // Checks if motor current draw is greater than the given threshold for cubes and if rollers are
  // intaking
  // Last condition prevents current spikes caused by starting to run intake from triggering this
  val hasCone: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CONE_CURRENT_THRESHOLD &&
        (
          rollerState == ManipulatorConstants.RollerStates.CONE_IN ||
            rollerState == ManipulatorConstants.RollerStates.CONE_IDLE
          ) &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  var lastIntakeSpikeTime = Clock.fpgaTime

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= ManipulatorConstants.ARM_MAX_EXTENSION

  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= ManipulatorConstants.ARM_MAX_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.armPosition >= ManipulatorConstants.ARM_OPEN_LOOP_SOFTLIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.armPosition <= ManipulatorConstants.ARM_OPEN_LOOP_SOFTLIMIT_RETRACTION

  var desiredArmPosition = ManipulatorConstants.ArmStates.MIN_EXTENSION.position

  val isAtCommandedPosition: Boolean
    get() {
      return (
        (inputs.armPosition - desiredArmPosition).absoluteValue <
          ManipulatorConstants.ARM_TOLERANCE
        )
    }

  val actualArmStates = hashMapOf<ManipulatorConstants.ArmStates, LoggedTunableValue<Meter>>()

  var armConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.ARM_MAX_VELOCITY, ManipulatorConstants.ARM_MAX_ACCELERATION
    )

  var prevArmSetpoint: TrapezoidProfile.State<Meter> = TrapezoidProfile.State()

  var isHomed = false

  /**
   * Uses the calculations via feed forward to determine where to set the arm position to.
   *
   * @param setpoint The trapezoidal profile containing the desired position of the arm.
   */
  fun setArmPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val armAcceleration =
      (setpoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setpoint

    val feedforward = armFeedforward.calculate(setpoint.velocity, armAcceleration)

    if ((forwardLimitReached && setpoint.velocity > 0.inches.perSecond) ||
      (reverseLimitReached && setpoint.velocity < 0.inches.perSecond)
    ) {
      // TODO: hold position func
      io.setArmVoltage(0.volts)
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }
    Logger.getInstance().recordOutput("Manipulator/armTargetPosition", setpoint.position.inInches)
    Logger.getInstance()
      .recordOutput("Manipulator/armTargetVelocity", setpoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Manipulator/armAcceleraction", armAcceleration.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Manipulator/armFeedforward", feedforward.inVolts)
  }

  init {
    ManipulatorConstants.ArmStates.values().forEach {
      actualArmStates[it] =
        LoggedTunableValue(
          "Manipulator/${it.name}", it.position, Pair({ it.inInches }, { it.inches })
        )
    }

    if (RobotBase.isReal()) {
      kP.initDefault(ManipulatorConstants.REAL_ARM_KP)
      kI.initDefault(ManipulatorConstants.REAL_ARM_KI)
      kD.initDefault(ManipulatorConstants.REAL_ARM_KD)
    } else {
      kP.initDefault(ManipulatorConstants.SIM_ARM_KP)
      kI.initDefault(ManipulatorConstants.SIM_ARM_KI)
      kD.initDefault(ManipulatorConstants.SIM_ARM_KD)
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Manipulator", inputs)
    Logger.getInstance().recordOutput("Manipulator/rollerState", rollerState.name)
    Logger.getInstance().recordOutput("Manipulator/hasCube", hasCube)
    Logger.getInstance().recordOutput("Manipulator/hasCone", hasCone)
    Logger.getInstance().recordOutput("Manipulator/rollerRunTime", lastRollerRunTime.inSeconds)
    Logger.getInstance().recordOutput("Manipulator/rollerRunTime", lastIntakeSpikeTime.inSeconds)
    Logger.getInstance().recordOutput("Elevator/isAtCommandedPosition", isAtCommandedPosition)
  }

  /**
   * Sets the power of the roller in volts.
   *
   * @param rpm: The revolutions per minute of the roller as stated in radians per second.
   */
  fun setRollerPower(voltage: ElectricalPotential) {
    io.setRollerPower(voltage)
    Logger.getInstance().recordOutput("Manipulator/rollerTargetVoltage", voltage.inVolts)
  }

  /**
   * Based on the current voltage of the arm, it determines the amount of volts to set the arm to
   * accordingly.
   *
   * @param voltage The desired voltage of the arm.
   */
  fun setArmVoltage(voltage: ElectricalPotential) {
    if (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts) {
      io.setArmVoltage(0.volts)
    } else {
      io.setArmVoltage(voltage)
    }
  }

  /**
   * Sets the roller motor brake mode.
   *
   * @param brake A flag representing the state of the break mode and whether it should be on/off.
   */
  fun setRollerBrakeMode(brake: Boolean) {
    io.setRollerBrakeMode(brake)
  }

  /**
   * Sets the arm motor brake mode.
   *
   * @param brake A flag representing the state of the break mode and whether it should be on/off.
   */
  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
  }

  /** Holds the current arm position in place. */
  fun holdArmPosition(): Command {
    var positionToHold = inputs.armPosition
    val holdPositionCommand =
      runOnce {
        positionToHold = inputs.armPosition
        Logger.getInstance()
          .recordOutput("/Manipulator/holdPosition", positionToHold.inInches)
      }
        .andThen(
          run {
            io.setArmPosition(positionToHold, armFeedforward.calculate(0.meters.perSecond))
          }
        )

    holdPositionCommand.name = "ManipulatorHoldPositionCommand"
    return holdPositionCommand
  }

  /** Sets the arm's power to the desired voltage until the forward/reverse limits are reached. */
  fun openLoopControl(voltage: ElectricalPotential): Command {
    val openLoopArmCommand =
      run { setArmVoltage(voltage) }.until {
        forwardOpenLoopLimitReached && voltage > 0.volts ||
          reverseOpenLoopLimitReached && voltage < 0.volts
      }
    if (voltage > 0.volts) {
      openLoopArmCommand.name = "ManipulatorExtendArmOpenLoopCommand"
    } else if (voltage < 0.volts) {
      openLoopArmCommand.name = "ManipulatorRetractArmOpenLoopCommand"
    } else {
      openLoopArmCommand.name = "ManipulatorZeroVoltageOpenLoopCommand"
    }

    return openLoopArmCommand
  }

  /** Utilizes the trapezoidal profile of the arm to extend the arm by the desired length. */
  fun extendArmPosition(targetPosition: Supplier<Length>): Command {
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(targetPosition.get(), 0.meters.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )
    var startTime = Clock.fpgaTime

    val extendArmPositionCommand =
      runOnce {
        startTime = Clock.fpgaTime
        desiredArmPosition = targetPosition.get()
        Logger.getInstance().recordOutput("/Manipulator/isAtSetpoint", false)
        armProfile =
          TrapezoidProfile(
            armConstraints,
            TrapezoidProfile.State(targetPosition.get(), 0.meters.perSecond),
            TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
          )
      }
        .andThen(
          run {
            setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
            Logger.getInstance()
              .recordOutput(
                "/Manipulator/isAtSetpoint",
                (targetPosition.get() - inputs.armPosition).absoluteValue <=
                  ManipulatorConstants.ARM_TOLERANCE
              )
          }
            .until {
              armProfile.isFinished(Clock.fpgaTime - startTime) ||
                (
                  forwardLimitReached &&
                    targetPosition.get() > ManipulatorConstants.ARM_SOFTLIMIT_EXTENSION ||
                    reverseLimitReached &&
                    targetPosition.get() <
                    ManipulatorConstants.ARM_SOFTLIMIT_RETRACTION
                  )
            }
        )

    extendArmPositionCommand.name = "manipulatorExtendArmPositionCommand"
    return extendArmPositionCommand
  }

  // roller command factories

  /**
   * @return A command that sets roller to voltage for intaking a cone, command ends when cone is
   * detected
   */
  fun rollerIntakeCone(): Command {
    val rollerIntakeConeCommand =
      runOnce { lastRollerRunTime = Clock.fpgaTime }
        .andThen(
          run { setRollerPower(ManipulatorConstants.RollerStates.CONE_IN.voltage) }.until {
            hasCone
          }
        )
        .finallyDo { lastRollerState = ManipulatorConstants.RollerStates.CONE_IN }

    rollerIntakeConeCommand.name = "ManipulatorRollerIntakeConeCommand"
    return rollerIntakeConeCommand
  }

  /**
   * @return A command that sets roller to voltage for intaking a cube, command ends when cube is
   * detected
   */
  fun rollerIntakeCube(): Command {
    val rollerIntakeCubeCommand =
      runOnce { lastRollerRunTime = Clock.fpgaTime }
        .andThen(
          run { setRollerPower(ManipulatorConstants.RollerStates.CUBE_IN.voltage) }.until {
            hasCube
          }
        )
        .finallyDo { lastRollerState = ManipulatorConstants.RollerStates.CUBE_IN }

    rollerIntakeCubeCommand.name = "ManipulatorRollerIntakeCubeCommand"
    return rollerIntakeCubeCommand
  }

  /** @return A command that sets roller to voltage for outtaking a cone, has no end condition */
  fun rollerOuttakeCone(): Command {
    val rollerOuttakeConeCommand =
      run { setRollerPower(ManipulatorConstants.RollerStates.CONE_OUT.voltage) }.finallyDo {
        // no spin ensure roller idle state after outtake won't spin
        lastRollerState = ManipulatorConstants.RollerStates.NO_SPIN
      }

    rollerOuttakeConeCommand.name = "ManipulatorRollerOuttakeConeCommand"
    return rollerOuttakeConeCommand
  }

  /**
   * @return A command that sets roller to voltage for outtaking a cube, command ends when cube is
   * detected
   */
  fun rollerOuttakeCube(): Command {
    val rollerOuttakeCubeCommand =
      run { setRollerPower(ManipulatorConstants.RollerStates.CUBE_OUT.voltage) }.finallyDo {
        // no spin ensure roller idle state after outtake won't spin
        lastRollerState = ManipulatorConstants.RollerStates.NO_SPIN
      }

    rollerOuttakeCubeCommand.name = "ManipulatorRollerOuttakeCubeCommand"
    return rollerOuttakeCubeCommand
  }

  /**
   * @return A that sets rollers to their corresponding idle voltage based on whether the last
   * intake was a cone or cube
   */
  fun rollerIdle(): Command {
    val rollerIdleCommand = run {
      var idleState = ManipulatorConstants.RollerStates.NO_SPIN
      if (lastRollerState.voltage.sign == ManipulatorConstants.RollerStates.CONE_IN.voltage.sign) {
        idleState = ManipulatorConstants.RollerStates.CONE_IDLE
      } else if (lastRollerState.voltage.sign ==
        ManipulatorConstants.RollerStates.CUBE_IN.voltage.sign
      ) {
        idleState = ManipulatorConstants.RollerStates.CUBE_IDLE
      }

      setRollerPower(idleState.voltage)
      Logger.getInstance().recordOutput("/Manipulator/idleState", idleState.name)
    }

    rollerIdleCommand.name = "ManipulatorRollerIdleCommand"
    return rollerIdleCommand
  }

  /** @return A command that sets roller power to 0 volts */
  fun rollerNoSpin(): Command {
    val rollerNoSpinCommand = run { setRollerPower(0.volts) }

    rollerNoSpinCommand.name = "ManipulatorRollerNoSpinCommand"
    return rollerNoSpinCommand
  }

  /** @return A command that idles the rollers and holds the arm at the same time */
  fun manipulatorIdle(): Command {
    val manipulatorIdleCommand = ParallelCommandGroup(rollerIdle(), holdArmPosition())
    manipulatorIdleCommand.name = "ManipulatorIdleCommand"
    return manipulatorIdleCommand
  }

  // could get rid of this and just pass in command from robot container
  val rollerStateToCommand =
    hashMapOf<ManipulatorConstants.RollerStates, Command>(
      ManipulatorConstants.RollerStates.NO_SPIN to rollerNoSpin(),
      ManipulatorConstants.RollerStates.CONE_IN to rollerIntakeCone(),
      ManipulatorConstants.RollerStates.CUBE_IN to rollerIntakeCube(),
      ManipulatorConstants.RollerStates.CONE_IN to rollerOuttakeCone(),
      ManipulatorConstants.RollerStates.CUBE_OUT to rollerOuttakeCube()
    )

  /**
   * main command for manipulator that combines roller and arm actions
   *
   * @param rollerState the desired state for the manipulator rollers
   *
   * @param armState the desired state for the manipulator arm
   *
   * @return A parrallel command ground that sets the roller power based on the corresponding state
   * and extends the arm to the arm state the command ends once both commands have finished
   */
  fun manipulatorCommand(
    rollerState: ManipulatorConstants.RollerStates,
    armState: ManipulatorConstants.ArmStates
  ): Command {
    val rollerCommand = rollerStateToCommand[rollerState] ?: rollerIdle()

    val armPosition = Supplier { actualArmStates[armState]?.get() ?: armState.position }
    val armCommand = extendArmPosition(armPosition)
    val manipulatorCommand = ParallelCommandGroup(rollerCommand, armCommand)
    manipulatorCommand.name = "ManipulatorCombinedCommand"
    return manipulatorCommand
  }

  /**
   * Slowly retracts arm and detects when the arm stalls using stator current draw zeros the
   * encoders when the arm stalls
   * @return A command that zeros the arm at the beginning of teleop
   */
  fun homeArmCommand(): Command {
    val maybeHomeArmCommand =
      run { io.setArmVoltage(ManipulatorConstants.ARM_HOMING_APPLIED_VOLTAGE) }
        .until {
          isHomed ||
            inputs.armPosition > ManipulatorConstants.ARM_HOMING_POSITION_THESHOLD ||
            inputs.armStatorCurrent > ManipulatorConstants.ARM_HOMING_STALL_CURRENT
        }
        .finallyDo {
          io.zeroEncoder()
          isHomed = true
        }
    maybeHomeArmCommand.name = "ElevatorHomingCommand"
    return maybeHomeArmCommand
  }
}
