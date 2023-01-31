package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
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

  val currentArmState: ManipulatorConstants.ActualArmStates
    get() = ManipulatorConstants.DesiredArmStates.fromArmPositionToState(inputs.armPosition)

  var armConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.ARM_MAX_VELOCITY, ManipulatorConstants.ARM_MAX_ACCELERATION
    )

  var armSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  var prevArmSetpoint: TrapezoidProfile.State<Meter> = TrapezoidProfile.State()

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
    return run {
      io.setArmVoltage(armFeedforward.calculate(0.inches.perSecond))
      Logger.getInstance().recordOutput("/ActiveCommands/HoldArmPosition", true)
    }
      .finallyDo { Logger.getInstance().recordOutput("/ActiveCommands/HoldArmPosition", false) }
  }

  /** Sets the arm's power to the desired voltage until the forward/reverse limits are reached. */
  fun openLoopControl(voltage: ElectricalPotential): Command {
    return run { setArmVoltage(voltage) }.until {
      forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts
    }
  }

  /** Utilizes the trapezoidal profile of the arm to extend the arm by the desired length. */
  fun extendArmPosition(position: Length): Command {
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(position, 0.meters.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )
    var startTime = Clock.fpgaTime

    val extendArmPositionCommand =
      run {
        setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
        Logger.getInstance()
          .recordOutput(
            "/Manipulator/isAtSetpoint",
            (position - inputs.armPosition).absoluteValue <=
              ManipulatorConstants.ARM_TOLERANCE
          )
      }
        .beforeStarting(
          {
            startTime = Clock.fpgaTime
            Logger.getInstance().recordOutput("/Manipulator/isAtSetpoint", false)
            armProfile =
              TrapezoidProfile(
                armConstraints,
                TrapezoidProfile.State(position, 0.meters.perSecond),
                TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
              )
          },
          this
        )
        .until { armProfile.isFinished(Clock.fpgaTime - startTime) }

    extendArmPositionCommand.name = "manipulatorExtendArmPositionCommand"
    return extendArmPositionCommand
  }
}
