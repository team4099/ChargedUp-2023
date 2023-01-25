package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class Manipulator(val io: ManipulatorIO) : SubsystemBase() {
  val inputs = ManipulatorIO.ManipulatorIOInputs()
  // placement feedforward
  val armFeedforward =
    SimpleMotorFeedforward(
      ManipulatorConstants.ARM_KS, ManipulatorConstants.ARM_KV, ManipulatorConstants.ARM_KA
    )
  private val kP =
    LoggedTunableValue("Manipulator/kP", Pair({ it.inVoltsPerMeter }, { it.volts.perMeter }))
  private val kI =
    LoggedTunableValue(
      "Manipulator/kP", Pair({ it.inVoltsPerMeterSeconds }, { it.volts.perMeterSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Manipulator/kP", Pair({ it.inVoltsPerMeterPerSecond }, { it.volts.perMeterPerSecond })
    )

  var lastIntakeRunTime = Clock.fpgaTime

  var rollerState = ManipulatorConstants.RollerStates.IDLE
    get() {
      for (state in ManipulatorConstants.RollerStates.values()) {
        if ((state.voltage - inputs.rollerAppliedVoltage).absoluteValue <=
          ManipulatorConstants.ROLLER_POWER_TOLERANCE
        ) {
          return state
        }
      }
      return ManipulatorConstants.RollerStates.DUMMY
    }

  // checks if motor current draw is greater than given threshold and if rollers are intaking
  // last condition prevnts current spikes caused by starting to run intake from triggering this
  val hasCube: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CUBE_CURRENT_THRESHOLD &&
        rollerState == ManipulatorConstants.RollerStates.CUBE_IN &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
        ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
    }

  val hasCone: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CONE_CURRENT_THRESHOLD &&
        rollerState == ManipulatorConstants.RollerStates.CONE_IN &&
        (Clock.fpgaTime - lastIntakeRunTime) >=
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

    if ((forwardLimitReached && setPoint.velocity > 0.meters.perSecond) ||
      (reverseLimitReached && setPoint.velocity < 0.meters.perSecond)
    ) {
      // TODO: hold position func
      io.setArmVoltage(0.volts)
    } else {
      io.setPosition(setPoint.position, feedforward)
    }
  }

  init {
    // setter isn't called on initialization
    rollerState = rollerState
  }

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Intake", inputs)
    Logger.getInstance().recordOutput("Intake/rollerState", rollerState.name)
    Logger.getInstance().recordOutput("Intake/hasCube", hasCube)
    Logger.getInstance().recordOutput("Intake/hasCone", hasCone)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeRunTime.inSeconds)
    Logger.getInstance().recordOutput("Intake/extendTime", lastIntakeSpikeTime.inSeconds)
  }

  fun setRollerPower(voltage: ElectricalPotential) {
    io.setRollerPower(voltage)
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
      setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
      Logger.getInstance()
        .recordOutput(
          "/Manipulator/isAtSetpoint",
          (position - inputs.armPosition).absoluteValue <= ManipulatorConstants.ARM_TOLERANCE
        )
    }
      .beforeStarting(
        {
          Logger.getInstance().recordOutput("/Manipulator/isAtSetpoint", false)
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(position, 0.meters.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
          startTime = Clock.fpgaTime
        },
        this
      )
      .until { armProfile.isFinished(Clock.fpgaTime - startTime) }
  }
}
