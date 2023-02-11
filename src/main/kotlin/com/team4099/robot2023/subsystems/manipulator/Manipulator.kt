package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Volt
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
import com.team4099.robot2023.subsystems.superstructure.RequestStructure.ManipulatorRequest as ManipulatorRequest

class Manipulator(val io: ManipulatorIO) : SubsystemBase() {
  val inputs = ManipulatorIO.ManipulatorIOInputs()
  // placement feedforward
  private var armFeedforward: SimpleMotorFeedforward<Meter, Volt>
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

  object TunableManipulatorStates {
    val minExtension =
      LoggedTunableValue(
        "Manipulator/minExtension",
        ManipulatorConstants.MIN_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )
    val maxExtension =
      LoggedTunableValue(
        "Manipulator/maxExtension",
        ManipulatorConstants.MAX_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )
    val shelfIntakeExtension =
      LoggedTunableValue(
        "Manipulator/shelfIntakeExtension",
        ManipulatorConstants.SHELF_INTAKE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val lowScoreExtension =
      LoggedTunableValue(
        "Manipulator/lowScoreExtension",
        ManipulatorConstants.LOW_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val midScoreExtension =
      LoggedTunableValue(
        "Manipulator/midScoreExtension",
        ManipulatorConstants.MID_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val highScoreExtension =
      LoggedTunableValue(
        "Manipulator/highScoreExtension",
        ManipulatorConstants.HIGH_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val coneIdleVoltage =
      LoggedTunableValue(
        "Manipulator/coneIdleVoltage",
        ManipulatorConstants.CONE_IDLE,
        Pair({ it.inVolts }, { it.volts })
      )

    val cubeIdleVoltage =
      LoggedTunableValue(
        "Manipulator/cubeIdleVoltage",
        ManipulatorConstants.CUBE_IDLE,
        Pair({ it.inVolts }, { it.volts })
      )

    val coneInVoltage =
      LoggedTunableValue(
        "Manipulator/coneInVoltage",
        ManipulatorConstants.CONE_IN,
        Pair({ it.inVolts }, { it.volts })
      )

    val cubeInVoltage =
      LoggedTunableValue(
        "Manipulator/cubeInVoltage",
        ManipulatorConstants.CUBE_IN,
        Pair({ it.inVolts }, { it.volts })
      )

    val coneOutVoltage =
      LoggedTunableValue(
        "Manipulator/coneOutVoltage",
        ManipulatorConstants.CONE_OUT,
        Pair({ it.inVolts }, { it.volts })
      )

    val cubeOutVoltage =
      LoggedTunableValue(
        "Manipulator/cubeOutVoltage",
        ManipulatorConstants.CUBE_OUT,
        Pair({ it.inVolts }, { it.volts })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Manipulator/openLoopExtendVoltage", 12.0.volts, Pair({ it.inVolts }, { it.volts })
      )

    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Manipulator/openLoopRetractVoltage", -12.0.volts, Pair({ it.inVolts }, { it.volts })
      )
  }

  // checks if motor current draw is greater than given threshold and if rollers are intaking
  // last condition prevnts current spikes caused by starting to run intake from triggering this
  var lastRollerRunTime = Clock.fpgaTime

  // Checks if motor current draw is greater than given threshold and if rollers are intaking
  // Last condition prevents current spikes caused by starting to run intake from triggering this
  val hasCube: Boolean
    get() {
      return inputs.rollerStatorCurrent >= ManipulatorConstants.CUBE_CURRENT_THRESHOLD &&
        (
          inputs.rollerAppliedVoltage == ManipulatorConstants.CUBE_IN ||
            inputs.rollerAppliedVoltage ==
            ManipulatorConstants
              .CUBE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
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
          inputs.rollerAppliedVoltage == ManipulatorConstants.CONE_IN ||
            inputs.rollerAppliedVoltage ==
            ManipulatorConstants
              .CONE_IDLE // TODO checking if their equal is gonna be wrong figure out a
          // better way
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

  var currentState: ManipulatorState = ManipulatorState.UNINITIALIZED

  var currentRequest: ManipulatorRequest =
    ManipulatorRequest.TargetingPosition(
      TunableManipulatorStates.minExtension.get(), ManipulatorConstants.IDLE_VOLTAGE
    )

  var armPositionTarget: Length = 0.0.inches

  var armVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastArmPositionTarget = 0.0.inches

  private var lastArmVoltage = 0.0.volts

  private var armConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.ARM_MAX_VELOCITY, ManipulatorConstants.ARM_MAX_ACCELERATION
    )

  private var prevArmSetpoint: TrapezoidProfile.State<Meter> = TrapezoidProfile.State()

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var armProfile =
    TrapezoidProfile(
      armConstraints,
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond),
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond)
    )

  var isHomed = false

  init {
    TunableManipulatorStates

    if (RobotBase.isReal()) {
      kP.initDefault(ManipulatorConstants.REAL_ARM_KP)
      kI.initDefault(ManipulatorConstants.REAL_ARM_KI)
      kD.initDefault(ManipulatorConstants.REAL_ARM_KD)

      armFeedforward =
        SimpleMotorFeedforward(
          ManipulatorConstants.ARM_KS, ManipulatorConstants.ARM_KV, ManipulatorConstants.ARM_KA
        )
    } else {
      kP.initDefault(ManipulatorConstants.SIM_ARM_KP)
      kI.initDefault(ManipulatorConstants.SIM_ARM_KI)
      kD.initDefault(ManipulatorConstants.SIM_ARM_KD)

      armFeedforward =
        SimpleMotorFeedforward(
          0.0.volts, ManipulatorConstants.ARM_KV, ManipulatorConstants.ARM_KA
        )
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Manipulator", inputs)
    Logger.getInstance().recordOutput("Manipulator/currentState", currentState.name)
    Logger.getInstance().recordOutput("Manipulator/hasCube", hasCube)
    Logger.getInstance().recordOutput("Manipulator/hasCone", hasCone)
    Logger.getInstance().recordOutput("Manipulator/rollerRunTime", lastRollerRunTime.inSeconds)
    Logger.getInstance().recordOutput("Manipulator/rollerRunTime", lastIntakeSpikeTime.inSeconds)

    Logger.getInstance().recordOutput("Manipulator/currentState", currentState.name)

    Logger.getInstance()
      .recordOutput("Manipulator/requestedState", currentRequest.javaClass.simpleName)

    Logger.getInstance()
      .recordOutput(
        "Manipulator/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
      )

    Logger.getInstance().recordOutput("Manipulator/armPositionTarget", armPositionTarget.inInches)

    Logger.getInstance().recordOutput("Manipulator/armVoltageTarget", armVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("Manipulator/rollerVoltageTarget", rollerVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("Manipulator/lastCommandedAngle", lastArmPositionTarget.inInches)

    Logger.getInstance().recordOutput("Manipulator/forwardLimitReached", forwardLimitReached)

    Logger.getInstance().recordOutput("Manipulator/reverseLimitReached", reverseLimitReached)

    var nextState = currentState
    when (currentState) {
      ManipulatorState.UNINITIALIZED -> {
        // Outputs
        // No designated output functionality because targeting position will take care of it next
        // loop cycle

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      ManipulatorState.OPEN_LOOP_REQUEST -> {
        // Outputs
        setArmVoltage(armVoltageTarget)
        setRollerPower(rollerVoltageTarget)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      ManipulatorState.TARGETING_POSITION -> {
        // Outputs
        if (armPositionTarget != lastArmPositionTarget) {
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(armPositionTarget, 0.0.inches.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
          timeProfileGeneratedAt = Clock.fpgaTime

          // This statement is only run when the armPositionTarget is first noticed to be different
          // than the previous setpoint the arm went to.
          lastArmPositionTarget = armPositionTarget
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        setArmPosition(armProfile.calculate(timeElapsed))
        setRollerPower(rollerVoltageTarget)

        Logger.getInstance()
          .recordOutput("GroundIntake/completedMotionProfile", armProfile.isFinished(timeElapsed))

        // Transitions
        nextState = fromRequestToState(currentRequest)

        // if we're transitioning out of targeting position, we want to make sure the next time we
        // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
        // the previous time we ran it)
        if (!(currentState.equivalentToRequest(currentRequest))) {
          // setting the last target to something unreasonable so the profile is generated next loop
          // cycle
          lastArmPositionTarget = (-1337).inches
        }
      }
      ManipulatorState.HOME -> {
        // Outputs
        if (!isHomed &&
          inputs.armPosition < ManipulatorConstants.ARM_HOMING_POSITION_THESHOLD &&
          inputs.armStatorCurrent < ManipulatorConstants.ARM_HOMING_STALL_CURRENT
        ) {
          setArmVoltage(ManipulatorConstants.ARM_HOMING_APPLIED_VOLTAGE)
        } else {
          zeroEncoder()
          isHomed = true
        }

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.
    currentState = nextState

    // Taking advantage of Kotlin's smart casting on val assignment.
    // https://kotlinlang.org/docs/typecasts.html#smart-casts
    when (val typedRequest = currentRequest) {
      is ManipulatorRequest.TargetingPosition -> armPositionTarget = typedRequest.position
      is ManipulatorRequest.OpenLoop -> armVoltageTarget = typedRequest.voltage
      is ManipulatorRequest.Home -> {
        armPositionTarget = -1337.inches
        armVoltageTarget = 0.0.volts
      }
    }
  }

  fun zeroEncoder() {
    io.zeroEncoder()
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

  fun openLoopExtendCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.OpenLoop(
          TunableManipulatorStates.openLoopExtendVoltage.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
    }

    returnCommand.name = "ManipulatorOpenLoopExtendCommand"
    return returnCommand
  }

  fun openLoopRetractCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.OpenLoop(
          TunableManipulatorStates.openLoopRetractVoltage.get(),
          ManipulatorConstants.IDLE_VOLTAGE
        )
    }

    returnCommand.name = "ManipulatorOpenLoopRetractCommand"
    return returnCommand
  }

  fun goToMinExtensionCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.minExtension.get(), ManipulatorConstants.IDLE_VOLTAGE
        )
    }

    returnCommand.name = "GoToMinExtensionCommand"
    return returnCommand
  }

  fun goToMaxExtensionCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.maxExtension.get(), ManipulatorConstants.IDLE_VOLTAGE
        )
    }

    returnCommand.name = "GoToMaxExtensionCommand"
    return returnCommand
  }

  fun intakeCubeFromDoubleSubstationCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.shelfIntakeExtension.get(),
          TunableManipulatorStates.cubeInVoltage.get()
        )
    }

    returnCommand.name = "IntakeCubeFromDoubleSubstationCommand"
    return returnCommand
  }

  fun intakeConeFromDoubleSubstationCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.shelfIntakeExtension.get(),
          TunableManipulatorStates.coneInVoltage.get()
        )
    }

    returnCommand.name = "IntakeConeFromDoubleSubstationCommand"
    return returnCommand
  }

  fun scoreCubeAtHybridNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.lowScoreExtension.get(),
          TunableManipulatorStates.cubeOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreCubeAtHybridNodeCommand"
    return returnCommand
  }

  fun scoreConeAtHybridNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.lowScoreExtension.get(),
          TunableManipulatorStates.coneOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreConeAtHybridNodeCommand"
    return returnCommand
  }

  fun scoreConeAtMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.midScoreExtension.get(),
          TunableManipulatorStates.coneOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreConeAtMidNodeCommand"
    return returnCommand
  }

  fun scoreCubeAtMidNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.midScoreExtension.get(),
          TunableManipulatorStates.cubeOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreCubeAtMidNodeCommand"
    return returnCommand
  }

  fun scoreCubeAtHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.highScoreExtension.get(),
          TunableManipulatorStates.cubeOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreConeAtHighNodeCommand"
    return returnCommand
  }

  fun scoreConeAtHighNodeCommand(): CommandBase {
    val returnCommand = runOnce {
      currentRequest =
        ManipulatorRequest.TargetingPosition(
          TunableManipulatorStates.highScoreExtension.get(),
          TunableManipulatorStates.coneOutVoltage.get()
        )
    }

    returnCommand.name = "ScoreConeAtHighNodeCommand"
    return returnCommand
  }

  companion object {
    enum class ManipulatorState {
      UNINITIALIZED,
      TARGETING_POSITION,
      OPEN_LOOP_REQUEST,
      HOME;

      inline fun equivalentToRequest(request: ManipulatorRequest): Boolean {
        return (
          (request is ManipulatorRequest.OpenLoop && this == OPEN_LOOP_REQUEST) ||
            (request is ManipulatorRequest.TargetingPosition && this == TARGETING_POSITION) ||
            (request is ManipulatorRequest.Home && this == HOME)
          )
      }
    }

    inline fun fromRequestToState(request: ManipulatorRequest): ManipulatorState {
      return when (request) {
        is ManipulatorRequest.OpenLoop -> ManipulatorState.OPEN_LOOP_REQUEST
        is ManipulatorRequest.TargetingPosition -> ManipulatorState.TARGETING_POSITION
        is ManipulatorRequest.Home -> ManipulatorState.HOME
      }
    }
  }
}
