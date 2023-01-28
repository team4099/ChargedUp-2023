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
import org.team4099.lib.units.AngularVelocity
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
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchPerSecond
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
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

  // TODO(fix units library and get rid of /3600)
  private val ROLLER_KV =
    LoggedTunableValue(
      "Manipulator/RollerKV",
      Pair(
        { it.inVoltsPerRotationsPerMinute / 3600 }, { it.volts / (1.0.rotations.perMinute) }
      )
    )

  var lastRollerRunTime = Clock.fpgaTime

  var rollerState = ManipulatorConstants.RollerStates.NO_SPIN
    get() {
      for (state in ManipulatorConstants.RollerStates.values()) {
        if ((state.velocity - inputs.rollerVelocity).absoluteValue <=
          ManipulatorConstants.ROLLER_SPEED_TOLERANCE
        ) {
          return state
        }
      }
      return ManipulatorConstants.RollerStates.DUMMY
    }

  var lastRollerState = rollerState

  // checks if motor current draw is greater than given threshold and if rollers are intaking
  // last condition prevnts current spikes caused by starting to run intake from triggering this
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
    get() {
      for (state in ManipulatorConstants.DesiredArmStates.values()) {
        if ((state.position - inputs.armPosition).absoluteValue <=
          ManipulatorConstants.ARM_TOLERANCE
        ) {
          return ManipulatorConstants.ActualArmStates.fromDesiredState(state)
        }
      }

      return ManipulatorConstants.ActualArmStates.BETWEEN_TWO_STATES
    }

  var armConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.ARM_MAX_VELOCITY, ManipulatorConstants.ARM_MAX_ACCELERATION
    )

  var armSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  var prevArmSetpoint: TrapezoidProfile.State<Meter> = TrapezoidProfile.State()
  fun setArmPosition(setPoint: TrapezoidProfile.State<Meter>) {
    val armAcceleration =
      (setPoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setPoint

    val feedforward = armFeedforward.calculate(setPoint.velocity, armAcceleration)

    if ((forwardLimitReached && setPoint.velocity > 0.inches.perSecond) ||
      (reverseLimitReached && setPoint.velocity < 0.inches.perSecond)
    ) {
      // TODO: hold position func
      io.setArmVoltage(0.volts)
    } else {
      io.setPosition(setPoint.position, feedforward)
    }
    Logger.getInstance().recordOutput("Manipulator/armTargetPosition", setPoint.position.inInches)
    Logger.getInstance()
      .recordOutput("Manipulator/armTargetVelocity", setPoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Manipulator/armAcceleraction", armAcceleration.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Manipulator/armFeedForward", feedforward.inVolts)
  }

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(ManipulatorConstants.REAL_ARM_KP)
      kI.initDefault(ManipulatorConstants.REAL_ARM_KI)
      kD.initDefault(ManipulatorConstants.REAL_ARM_KD)
      ROLLER_KV.initDefault(ManipulatorConstants.REAL_ROLLER_KV)
    } else {
      kP.initDefault(ManipulatorConstants.SIM_ARM_KP)
      kI.initDefault(ManipulatorConstants.SIM_ARM_KI)
      kD.initDefault(ManipulatorConstants.SIM_ARM_KD)
      ROLLER_KV.initDefault(ManipulatorConstants.SIM_ROLLER_KV)
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

  fun setRollerPower(rpm: AngularVelocity) {
    val voltage =
      (rpm.inRotationsPerMinute * ROLLER_KV.get().inVoltsPerRotationsPerMinute / 3600).volts

    io.setRollerPower(voltage)
    Logger.getInstance().recordOutput("Manipulator/rollerTargetSpeedRPM", rpm.inRotationsPerMinute)
    Logger.getInstance().recordOutput("Manipulator/rollerTargetVoltage", voltage.inVolts)
  }

  fun setArmVoltage(voltage: ElectricalPotential) {
    if (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts) {
      io.setArmVoltage(0.volts)
    } else {
      io.setArmVoltage(voltage)
    }
  }

  fun setRollerBrakeMode(brake: Boolean) {
    io.setRollerBrakeMode(brake)
  }

  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
  }

  fun holdArmPosition(): Command {
    return run {
      io.setArmVoltage(armFeedforward.calculate(0.inches.perSecond))
      Logger.getInstance().recordOutput("/ActiveCommands/HoldArmPosition", true)
    }
      .finallyDo { Logger.getInstance().recordOutput("/ActiveCommands/HoldArmPosition", false) }
  }

  fun openLoopControl(voltage: ElectricalPotential): Command {
    return run { setArmVoltage(voltage) }.until {
      forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts
    }
  }

  fun extendArmPosition(position: Length): Command {
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(position, 0.meters.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )
    var startTime = Clock.fpgaTime

    return run {
      Logger.getInstance().recordOutput("/ActiveCommands/ExtendArmPosition", true)

      setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
      Logger.getInstance()
        .recordOutput(
          "/Manipulator/isAtSetpoint",
          (position - inputs.armPosition).absoluteValue <= ManipulatorConstants.ARM_TOLERANCE
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
      .finallyDo { Logger.getInstance().recordOutput("/ActiveCommands/ExtendArmPosition", false) }
  }
}
