package com.team4099.robot2023.subsystems.manipulator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
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
import java.util.function.Consumer
import com.team4099.robot2023.subsystems.superstructure.Request.ManipulatorRequest as ManipulatorRequest

class Manipulator(val io: ManipulatorIO, ) {
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

  private val filterSize = LoggedTunableNumber("Manipulator/filterSize", 5.0)

  val isStowed: Boolean
    get() =
      (inputs.armPosition - TunableManipulatorStates.minExtension.get()).absoluteValue <=
        ManipulatorConstants.ARM_TOLERANCE

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

    val intakeTime =
      LoggedTunableValue(
        "Manipulator/intakeTime",
        ManipulatorConstants.INTAKE_IN_TIME,
        Pair({ it.inSeconds }, { it.seconds })
      )

    val coneSpitTime =
      LoggedTunableValue(
        "Manipulator/coneSpitTime",
        ManipulatorConstants.CONE_SPIT_OUT_TIME,
        Pair({ it.inSeconds }, { it.seconds })
      )

    val cubeSpitTime =
      LoggedTunableValue(
        "Manipulator/cubeSpitTime",
        ManipulatorConstants.CUBE_SPIT_OUT_TIME,
        Pair({ it.inSeconds }, { it.seconds })
      )

    val groundIntakeCubeExtension =
      LoggedTunableValue(
        "Manipulator/groundIntakeCubeExtension",
        ManipulatorConstants.INTAKE_CUBE_FROM_GROUND_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val groundIntakeConeExtension =
      LoggedTunableValue(
        "Manipulator/groundIntakeCubeExtension",
        ManipulatorConstants.INTAKE_CONE_FROM_GROUND_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationIntakeShelfExtension =
      LoggedTunableValue(
        "Manipulator/singleSubstationIntakeShelfExtension",
        ManipulatorConstants.SINGLE_SUBSTATION_INTAKE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val doubleSubstationIntakeShelfExtension =
      LoggedTunableValue(
        "Manipulator/doubleSubstationIntakeShelfExtension",
        ManipulatorConstants.DOUBLE_SUBSTATION_SHELF_INTAKE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val lowCubeScoreExtension =
      LoggedTunableValue(
        "Manipulator/lowCubeScoreExtension",
        ManipulatorConstants.LOW_CUBE_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val midCubeScoreExtension =
      LoggedTunableValue(
        "Manipulator/midCubeScoreExtension",
        ManipulatorConstants.MID_CUBE_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val highCubeScoreExtension =
      LoggedTunableValue(
        "Manipulator/highCubeScoreExtension",
        ManipulatorConstants.HIGH_CUBE_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val lowConeScoreExtension =
      LoggedTunableValue(
        "Manipulator/lowConeScoreExtension",
        ManipulatorConstants.LOW_CONE_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val midConeScoreExtension =
      LoggedTunableValue(
        "Manipulator/midConeScoreExtension",
        ManipulatorConstants.MID_CONE_SCORE_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val highConeScoreExtension =
      LoggedTunableValue(
        "Manipulator/highConeScoreExtension",
        ManipulatorConstants.HIGH_CONE_SCORE_EXTENSION,
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
        "Manipulator/openLoopExtendVoltage", 0.5.volts, Pair({ it.inVolts }, { it.volts })
      )

    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Manipulator/openLoopRetractVoltage", -12.0.volts, Pair({ it.inVolts }, { it.volts })
      )
  }

  // checks if motor current draw is greater than given threshold and if rollers are intaking
  // last condition prevents current spikes caused by starting to run intake from triggering this
  var lastRollerRunTime = Clock.fpgaTime

  var lastHeldGamePieceDetected = Clock.fpgaTime

  var filterValue: Double = 0.0

  // Checks if motor current draw is greater than given threshold and if rollers are intaking
  // Last condition prevents current spikes caused by starting to run intake from triggering this
  val hasCube: Boolean
    get() {
      filterValue = inputs.rollerStatorCurrent.inAmperes
      return (
        filterValue >= ManipulatorConstants.CUBE_CURRENT_THRESHOLD.inAmperes &&
          (Clock.fpgaTime - lastRollerRunTime) >= ManipulatorConstants.INTAKE_IN_TIME
        ) ||
        inputs.isSimulating
    }

  val grippingCone: Boolean
    get() {
      return (
        hasCone &&
          ((Clock.fpgaTime - lastHeldGamePieceDetected) >= ManipulatorConstants.INTAKE_IN_TIME)
        )
    }

  // Checks if motor current draw is greater than the given threshold for cubes and if rollers are
  // intaking
  // Last condition prevents current spikes caused by starting to run intake from triggering this
  val hasCone: Boolean
    get() {
      return (
        inputs.rollerStatorCurrent.inAmperes >=
          ManipulatorConstants.CONE_CURRENT_THRESHOLD.inAmperes &&
          (Clock.fpgaTime - lastRollerRunTime) >=
          ManipulatorConstants.MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE
        ) ||
        inputs.isSimulating
    }

  val holdingGamePiece: GamePiece
    get() {
      if (hasCone) {
        AutonomousSelector.hasCone.setBoolean(true)
        return GamePiece.CONE
      }
      if (hasCube) {
        AutonomousSelector.hasCube.setBoolean(true)
        return GamePiece.CUBE
      }
      AutonomousSelector.hasCone.setBoolean(false)
      AutonomousSelector.hasCube.setBoolean(false)
      return GamePiece.NONE
    }

  var lastHeldGamePiece = GamePiece.NONE

  var rumbleTrigger = false

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
    ManipulatorRequest.OpenLoop(0.0.volts, ManipulatorConstants.IDLE_VOLTAGE)
    set(value) {
        when (value) {
          is ManipulatorRequest.OpenLoop -> {
            armVoltageTarget = value.voltage
            rollerVoltageTarget = value.rollerVoltage
          }
          is ManipulatorRequest.TargetingPosition -> {
            armPositionTarget = value.position
            rollerVoltageTarget = value.rollerVoltage
          }
          else -> {}
        }
        field = value
      }

  var armPositionTarget: Length = 0.0.inches

  var armVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastArmPositionTarget = 0.0.inches

  private var lastArmVoltage = 0.0.volts

  private var lastRollerVoltage = 0.0.volts

  private var armConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ManipulatorConstants.ARM_MAX_VELOCITY, ManipulatorConstants.ARM_MAX_ACCELERATION
    )

  private var prevArmSetpoint: TrapezoidProfile.State<Meter> = TrapezoidProfile.State()

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var lastHomingStatorCurrentTripTime = -1337.seconds

  private var armProfile =
    TrapezoidProfile(
      armConstraints,
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond),
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond)
    )

  var isHomed = false

  val isAtTargetedPosition: Boolean
    get() =
      currentRequest is ManipulatorRequest.TargetingPosition &&
        armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
        (inputs.armPosition - armPositionTarget).absoluteValue <=
        ManipulatorConstants.ARM_TOLERANCE

  var lastDropTime = Clock.fpgaTime

  val rumbleTime = 0.5.seconds

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

  fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    var updateCone = hasCone
    var updateCube = hasCube

    if (lastHeldGamePiece != holdingGamePiece && !rumbleTrigger) {
      rumbleTrigger = true
      lastDropTime = Clock.fpgaTime
    }

    if (Clock.fpgaTime - lastDropTime > rumbleTime) {
      rumbleTrigger = false
    }

    lastHeldGamePiece = holdingGamePiece

    Logger.getInstance().recordOutput("Manipulator/gamePieceRumble", rumbleTrigger)

    Logger.getInstance().recordOutput("Manipulator/filteredStatorRoller", filterValue)

    /*
    if (filterSize.hasChanged()) {
      rollerStatorCurrentFilter =
        LinearFilter.highPass(filterSize.get(), ManipulatorConstants.FILTER_PERIOD.inMilliseconds)
    }
    */

    Logger.getInstance().processInputs("Manipulator", inputs)

    Logger.getInstance().recordOutput("Manipulator/currentState", currentState.name)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.getInstance().recordOutput("Manipulator/isHomed", isHomed)

      Logger.getInstance().recordOutput("Manipulator/hasCube", hasCube)

      Logger.getInstance().recordOutput("Manipulator/hasCone", hasCone)

      Logger.getInstance().recordOutput("Manipulator/isAtTargetedPosition", isAtTargetedPosition)

      Logger.getInstance()
        .recordOutput("Manipulator/lastRollerRunTime", lastRollerRunTime.inSeconds)

      Logger.getInstance()
        .recordOutput("Manipulator/lastRollerSpikeTime", lastIntakeSpikeTime.inSeconds)

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
        .recordOutput("Manipulator/lastRollerVoltageTarget", lastRollerVoltage.inVolts)

      Logger.getInstance()
        .recordOutput("Manipulator/rollerVoltageTarget", rollerVoltageTarget.inVolts)

      Logger.getInstance()
        .recordOutput("Manipulator/lastCommandedArmPosition", lastArmPositionTarget.inInches)

      Logger.getInstance().recordOutput("Manipulator/forwardLimitReached", forwardLimitReached)

      Logger.getInstance().recordOutput("Manipulator/reverseLimitReached", reverseLimitReached)

      /*
      Logger.getInstance()
        .recordOutput(
          "Manipulator/filteredRollerMotorStatorCurrent",
          rollerStatorCurrentFilter.calculate(inputs.rollerStatorCurrent.inAmperes)
        )
       */
    }

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
        lastRollerRunTime = Clock.fpgaTime

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      ManipulatorState.TARGETING_POSITION -> {
        setRollerPower(rollerVoltageTarget)
        if (lastRollerVoltage != rollerVoltageTarget) {
          lastRollerRunTime = Clock.fpgaTime
          lastRollerVoltage = rollerVoltageTarget
        }

        // Outputs
        if (armPositionTarget != lastArmPositionTarget) {

          val preProfileGenerate = Clock.realTimestamp
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(armPositionTarget, 0.0.inches.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
          val postProfileGenerate = Clock.realTimestamp
          Logger.getInstance()
            .recordOutput(
              "/Manipulator/ProfileGenerationMS",
              postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
            )

          timeProfileGeneratedAt = Clock.fpgaTime

          // This statement is only run when the armPositionTarget is first noticed to be different
          // than the previous setpoint the arm went to.
          lastArmPositionTarget = armPositionTarget
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        setArmPosition(armProfile.calculate(timeElapsed))

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
          lastArmVoltage = -1337.volts
          lastRollerVoltage = -1337.volts
        }
      }
      ManipulatorState.HOME -> {
        // Outputs

        if (inputs.armStatorCurrent < ManipulatorConstants.ARM_HOMING_STALL_CURRENT) {
          lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }
        if (!inputs.isSimulating &&
          (
            !isHomed &&
              inputs.armStatorCurrent < ManipulatorConstants.ARM_HOMING_STALL_CURRENT &&
              (Clock.fpgaTime - lastHomingStatorCurrentTripTime) <
              ManipulatorConstants.HOMING_STALL_TIME_THRESHOLD
            )
        ) {
          setArmVoltage(ManipulatorConstants.ARM_HOMING_APPLIED_VOLTAGE)
        } else {
          zeroEncoder()
          isHomed = true
        }

        if (isHomed) {
          // Transitions
          nextState = fromRequestToState(currentRequest)
        }
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.
    currentState = nextState

    // Taking advantage of Kotlin's smart casting on val assignment.
    // https://kotlinlang.org/docs/typecasts.html#smart-casts
    when (val typedRequest = currentRequest) {
      is ManipulatorRequest.TargetingPosition -> {
        armPositionTarget = typedRequest.position
        rollerVoltageTarget = typedRequest.rollerVoltage
      }
      is ManipulatorRequest.OpenLoop -> {
        armVoltageTarget = typedRequest.voltage
        rollerVoltageTarget = typedRequest.rollerVoltage
      }
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

  fun regenerateProfileNextLoopCycle() {
    lastArmVoltage = -3337.volts
    lastArmPositionTarget = -3337.inches
    lastIntakeSpikeTime = -3337.seconds
    lastRollerRunTime = -3337.seconds
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
    Logger.getInstance()
      .recordOutput("Manipulator/profilePositionInches", setpoint.position.inInches)
    Logger.getInstance()
      .recordOutput(
        "Manipulator/profileVelocityInchesPerSecond", setpoint.velocity.inInchesPerSecond
      )
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
    if (isHomed &&
      (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts)
    ) {
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
