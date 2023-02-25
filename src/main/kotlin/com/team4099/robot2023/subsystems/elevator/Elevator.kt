package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.MechanismSimConstants.carriageAttachment
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteXPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.elevatorAbsoluteYPosition
import com.team4099.robot2023.config.constants.MechanismSimConstants.secondStageAttachment
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import java.lang.Math.sin
import com.team4099.robot2023.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest

class Elevator(val io: ElevatorIO) {
  val inputs = ElevatorIO.ElevatorInputs()

  private var elevatorFeedforwardFirstStage: ElevatorFeedforward = ElevatorFeedforward(
    ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
    ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
    ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
    ElevatorConstants.ELEVATOR_KA_FIRST_STAGE
  )
  private var elevatorFeedforwardSecondStage: ElevatorFeedforward = ElevatorFeedforward(
    ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
    ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
    ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
    ElevatorConstants.ELEVATOR_KA_SECOND_STAGE
  )

  // PID and Feedforward Values
  var elevatorFeedforward: ElevatorFeedforward = elevatorFeedforwardFirstStage
    get() {
      return if (inputs.elevatorPosition > ElevatorConstants.FIRST_STAGE_HEIGHT) {
        elevatorFeedforwardSecondStage
      } else {
        elevatorFeedforwardFirstStage
      }
    }

  private val kP =
    LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  private val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  object TunableElevatorHeights {
    val minPosition =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION,
        Pair({ it.inInches }, { it.inches })
      )

    val maxPosition =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION,
        Pair({ it.inInches }, { it.inches })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage", 8.volts, Pair({ it.inVolts }, { it.volts })
      )

    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage", -12.0.volts, Pair({ it.inVolts }, { it.volts })
      )

    val cubeDropPosition =
      LoggedTunableValue(
        "Elevator/cubeDropPosition",
        ElevatorConstants.CUBE_DROP_POSITION_DELTA,
        Pair({ it.inInches }, { it.inches })
      )

    val coneDropPosition =
      LoggedTunableValue(
        "Elevator/coneDropPosition",
        ElevatorConstants.CONE_DROP_POSITION_DELTA,
        Pair({ it.inInches }, { it.inches })
      )

    val slamVelocity =
      LoggedTunableValue(
        "Elevator/slamVelocity",
        ElevatorConstants.SLAM_VELOCITY,
        Pair({ it.inInchesPerSecond }, { it.inches.perSecond })
      )

    val shelfIntakeCubeOffset =
      LoggedTunableValue(
        "Elevator/shelfIntakeCubeOffset",
        ElevatorConstants.DOUBLE_SUBSTATION_CUBE_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val shelfIntakeConeOffset =
      LoggedTunableValue(
        "Elevator/shelfIntakeConeOffset",
        ElevatorConstants.DOUBLE_SUBSTATION_CONE_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationCubeOffset =
      LoggedTunableValue(
        "Elevator/singleSubstationCubeOffset",
        ElevatorConstants.DOUBLE_SUBSTATION_CUBE_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationConeOffset =
      LoggedTunableValue(
        "Elevator/singleSubstationConeOffset",
        ElevatorConstants.DOUBLE_SUBSTATION_CONE_OFFSET,
        Pair({ it.inInches }, { it.inches })
      )

    val hybridHeight =
      LoggedTunableValue(
        "Elevator/hybridHeight", 0.0.inches, Pair({ it.inInches }, { it.inches })
      )

    val midCubeHeight =
      LoggedTunableValue(
        "Elevator/midCubeHeight",
        fromHeightToPosition(FieldConstants.Grids.midCubeZ),
        Pair({ it.inInches }, { it.inches })
      )

    val midConeHeight =
      LoggedTunableValue(
        "Elevator/midConeHeight",
        fromHeightToPosition(FieldConstants.Grids.midConeZ),
        Pair({ it.inInches }, { it.inches })
      )

    val highCubeHeight =
      LoggedTunableValue(
        "Elevator/highCubeHeight",
        fromHeightToPosition(FieldConstants.Grids.highCubeZ),
        Pair({ it.inInches }, { it.inches })
      )

    val highConeHeight =
      LoggedTunableValue(
        "Elevator/highConeHeight",
        fromHeightToPosition(FieldConstants.Grids.highConeZ),
        Pair({ it.inInches }, { it.inches })
      )

    val singleSubstationHeight =
      LoggedTunableValue(
        "Elevator/singleSubstationHeight",
        fromHeightToPosition(FieldConstants.LoadingZone.singleSubstationHeight),
        Pair({ it.inInches }, { it.inches })
      )

    val doubleSubstationHeight =
      LoggedTunableValue(
        "Elevator/doubleSubstationHeight",
        fromHeightToPosition(FieldConstants.LoadingZone.doubleSubstationShelfZ),
        Pair({ it.inInches }, { it.inches })
      )

    val xPos =
      LoggedTunableValue(
        "Elevator/xPos",
        0.0.meters,
      )
    val yPos =
      LoggedTunableValue(
        "Elevator/yPos",
        0.0.meters,
      )
    val zPos =
      LoggedTunableValue(
        "Elevator/zPos",
        0.0.meters,
      )

    val thetaPos =
      LoggedTunableValue("Elevator/thetaPos", 0.0.degrees, Pair({ it.inDegrees }, { it.degrees }))

    val x1Pos =
      LoggedTunableValue(
        "Elevator/x1Pos",
        0.0.meters,
      )
    val y1Pos =
      LoggedTunableValue(
        "Elevator/y1Pos",
        0.0.meters,
      )
    val z1Pos =
      LoggedTunableValue(
        "Elevator/z1Pos",
        0.0.meters,
      )

    val theta1Pos =
      LoggedTunableValue(
        "Elevator/theta1Pos",
        ElevatorConstants.ELEVATOR_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
  }

  val forwardLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION
  val reverseLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION

  val forwardOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION
  val reverseOpenLoopLimitReached: Boolean
    get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION

  val isStowed: Boolean
    get() =
      inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_MAX_RETRACTION + ElevatorConstants.ELEVATOR_TOLERANCE

  var isHomed = false

  var currentState: ElevatorState = ElevatorState.UNINITIALIZED

  var currentRequest: ElevatorRequest =
    ElevatorRequest.OpenLoop(0.0.volts)
    set(value) {
        when (value) {
          is ElevatorRequest.OpenLoop -> elevatorVoltageTarget = value.voltage
          is ElevatorRequest.TargetingPosition -> {
            elevatorPositionTarget = value.position
            elevatorVelocityTarget = value.finalVelocity
          }
          else -> {}
        }
        field = value
      }

  var elevatorPositionTarget = 0.0.inches
    private set

  var elevatorVelocityTarget = 0.0.inches.perSecond
    private set

  var elevatorVoltageTarget = 0.0.volts
    private set

  private var lastRequestedPosition = -1337.inches

  private var lastRequestedVelocity = -1337.inches.perSecond

  private var lastRequestedVoltage = -1337.volts

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var lastHomingStatorCurrentTripTime = -1337.seconds

  // trapezoidal profile constraints
  private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
    TrapezoidProfile.Constraints(
      ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION
    )

  private var elevatorSetpoint: TrapezoidProfile.State<Meter> =
    TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)

  private var elevatorProfile =
    TrapezoidProfile(
      elevatorConstraints,
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond),
      TrapezoidProfile.State(-1337.inches, -1337.inches.perSecond)
    )

  val isAtTargetedPosition: Boolean
    get() =
      currentRequest is ElevatorRequest.TargetingPosition &&
        elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
        (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
        ElevatorConstants.ELEVATOR_TOLERANCE

  init {
    TunableElevatorHeights

    // initializing pid constants and changing FF for sim vs real
    if (RobotBase.isReal()) {
      isHomed = false

      kP.initDefault(ElevatorConstants.REAL_KP)
      kI.initDefault(ElevatorConstants.REAL_KI)
      kD.initDefault(ElevatorConstants.REAL_KD)

      elevatorFeedforwardSecondStage =
        ElevatorFeedforward(
          ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KA_SECOND_STAGE
        )

      elevatorFeedforwardFirstStage =
        ElevatorFeedforward(
          ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KA_FIRST_STAGE
        )
    } else {
      isHomed = true

      kP.initDefault(ElevatorConstants.SIM_KP)
      kI.initDefault(ElevatorConstants.SIM_KI)
      kD.initDefault(ElevatorConstants.SIM_KD)

      elevatorFeedforwardSecondStage = // TODO make sim values
        ElevatorFeedforward(
          ElevatorConstants.REAL_ELEVATOR_KS_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KG_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KV_SECOND_STAGE,
          ElevatorConstants.ELEVATOR_KA_SECOND_STAGE
        )

      elevatorFeedforwardFirstStage =
        ElevatorFeedforward(
          ElevatorConstants.REAL_ELEVATOR_KS_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KG_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KV_FIRST_STAGE,
          ElevatorConstants.ELEVATOR_KA_FIRST_STAGE
        )

      io.configPID(kP.get(), kI.get(), kD.get())
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Elevator", inputs)

    Logger.getInstance()
      .recordOutput(
        "Elevator/elevatorHeightWithRespectToGround",
        ElevatorConstants.ElevatorStates.fromPositionToHeight(inputs.elevatorPosition).inInches
      )

    Logger.getInstance().recordOutput("Elevator/currentState", currentState.name)
    Logger.getInstance()
      .recordOutput("Elevator/currentRequest", currentRequest.javaClass.simpleName)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.getInstance().recordOutput("Elevator/isHomed", isHomed)

      Logger.getInstance().recordOutput("Elevator/isAtTargetPosition", isAtTargetedPosition)
      Logger.getInstance().recordOutput("Elevator/isStowed", isStowed)
      Logger.getInstance()
        .recordOutput("Elevator/lastGeneratedAt", timeProfileGeneratedAt.inSeconds)

      Logger.getInstance()
        .recordOutput("Elevator/elevatorPositionTarget", elevatorPositionTarget.inInches)
      Logger.getInstance()
        .recordOutput("Elevator/elevatorVelocityTarget", elevatorVelocityTarget.inInchesPerSecond)
      Logger.getInstance()
        .recordOutput("Elevator/elevatorVoltageTarget", elevatorVoltageTarget.inVolts)

      Logger.getInstance()
        .recordOutput("Elevator/lastElevatorPositionTarget", lastRequestedPosition.inInches)
      Logger.getInstance()
        .recordOutput(
          "Elevator/lastElevatorVelocityTarget", lastRequestedVelocity.inInchesPerSecond
        )
      Logger.getInstance()
        .recordOutput("Elevator/lastElevatorVoltageTarget", lastRequestedVoltage.inVolts)

      Logger.getInstance().recordOutput("Elevator/forwardLimitReached", forwardLimitReached)
      Logger.getInstance().recordOutput("Elevator/reverseLimitReached", reverseLimitReached)
    }

    var nextState = currentState
    when (currentState) {
      ElevatorState.UNINITIALIZED -> {
        // Outputs

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.OPEN_LOOP -> {
        // Outputs
        setOutputVoltage(elevatorVoltageTarget)

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)
      }
      ElevatorState.TARGETING_POSITION -> {
        // Outputs
        if (elevatorPositionTarget != lastRequestedPosition ||
          elevatorVelocityTarget != lastRequestedVelocity
        ) {
          elevatorProfile =
            TrapezoidProfile(
              elevatorConstraints,
              TrapezoidProfile.State(elevatorPositionTarget, elevatorVelocityTarget),
              TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
            )

          Logger.getInstance().recordOutput("Elevator/initialPosition", elevatorProfile.initial.position.inInches)
          Logger.getInstance().recordOutput("Elevator/initialVelocity", elevatorProfile.initial.velocity.inInchesPerSecond)
          timeProfileGeneratedAt = Clock.fpgaTime
          lastRequestedPosition = elevatorPositionTarget
          lastRequestedVelocity = elevatorVelocityTarget
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        val profilePosition = elevatorProfile.calculate(timeElapsed)

        setPosition(profilePosition)

        Logger.getInstance()
          .recordOutput(
            "Elevator/completedMotionProfile", elevatorProfile.isFinished(timeElapsed)
          )

        Logger.getInstance().recordOutput("Elevator/profileTargetPosition", profilePosition.position.inInches)
        Logger.getInstance().recordOutput("Elevator/profileTargetVelocity", profilePosition.velocity.inInchesPerSecond)

        // Transitions
        nextState = fromElevatorRequestToState(currentRequest)

        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastRequestedPosition = -1337.inches
          lastRequestedVelocity = -1337.inches.perSecond
        }
      }
      ElevatorState.HOME -> {
        // Outputs
        if (inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT){
          lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }
        if (!inputs.isSimulating && (!isHomed && inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT && (Clock.fpgaTime - lastHomingStatorCurrentTripTime) < ElevatorConstants.HOMING_STALL_TIME_THRESHOLD)) {
          setHomeVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
        } else {
          zeroEncoder()
          isHomed = true
        }

        // Transition
        if (isHomed){
          nextState = fromElevatorRequestToState(currentRequest)
        }
      }
    }

    currentState = nextState
  }

  /**
   * Sets the voltage of the elevator motors but also checks to make sure elevator doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setOutputVoltage(voltage: ElectricalPotential) {
    if (forwardLimitReached && voltage > 0.volts || reverseLimitReached && voltage < 0.volts) {
      io.setOutputVoltage(0.volts)
    } else {
      io.setOutputVoltage(voltage)
    }
  }

  private fun setHomeVoltage(voltage: ElectricalPotential){
    io.setOutputVoltage(voltage)
  }

  fun regenerateProfileNextLoopCycle(){
    lastRequestedPosition = -3337.inches
    lastRequestedVelocity = -3337.inches.perSecond
    lastRequestedVoltage = -3337.volts
  }

  /**
   * Sets the elevator to a specific position using trapezoidal profile state and feedforward also
   * has safety for max extension and retractions
   *
   * @param voltage the voltage to set the motor to
   */
  fun setPosition(setpoint: TrapezoidProfile.State<Meter>) {
    val elevatorAccel =
      ((setpoint.velocity - elevatorSetpoint.velocity) / (Constants.Universal.LOOP_PERIOD_TIME))

    elevatorSetpoint = setpoint

    val feedforward = elevatorFeedforward.calculate(setpoint.velocity, elevatorAccel)

    if (forwardLimitReached && setpoint.position > inputs.elevatorPosition ||
      reverseLimitReached && setpoint.position < inputs.elevatorPosition
    ) {
      io.setOutputVoltage(0.0.volts)
    } else {
      io.setPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("Elevator/targetPosition", setpoint.position.inInches)
    Logger.getInstance().recordOutput("Elevator/targetVel", setpoint.velocity.inInchesPerSecond)
    Logger.getInstance()
      .recordOutput("Elevator/elevatorAcceleration", elevatorAccel.inInchesPerSecondPerSecond)
    Logger.getInstance().recordOutput("Elevator/elevatorFeedForward", feedforward.inVolts)
  }

  /** set the current encoder position to be the encoders zero value */
  fun zeroEncoder() {
    io.zeroEncoder()
  }

  fun updateMech2d() {
    // updating carriage attachment based on height of elevator
    val secondStagePosition =
      clamp(inputs.elevatorPosition, 0.0.inches, ElevatorConstants.FIRST_STAGE_HEIGHT)

    val carriagePosition =
      clamp(
        inputs.elevatorPosition, secondStagePosition, ElevatorConstants.ELEVATOR_MAX_EXTENSION
      )

    secondStageAttachment.setPosition(
      (
        elevatorAbsoluteXPosition -
          ElevatorConstants.ElevatorStates.fromPositionToRange(secondStagePosition)
        )
        .inInches,
      (
        elevatorAbsoluteYPosition +
          ElevatorConstants.ElevatorStates.fromPositionToHeight(secondStagePosition)
        )
        .inInches
    )
    carriageAttachment.setPosition(
      (
        elevatorAbsoluteXPosition -
          ElevatorConstants.ElevatorStates.fromPositionToRange(carriagePosition)
        )
        .inInches,
      (
        elevatorAbsoluteYPosition +
          ElevatorConstants.ElevatorStates.fromPositionToHeight(carriagePosition)
        )
        .inInches
    )
  }

  companion object {
    enum class ElevatorState {
      UNINITIALIZED,
      TARGETING_POSITION,
      OPEN_LOOP,
      HOME;

      inline fun equivalentToRequest(request: ElevatorRequest): Boolean {
        return ((request is ElevatorRequest.Home) && this == HOME) ||
          (request is ElevatorRequest.OpenLoop && this == OPEN_LOOP) ||
          (request is ElevatorRequest.TargetingPosition && this == TARGETING_POSITION)
      }
    }

    inline fun fromElevatorRequestToState(request: ElevatorRequest): ElevatorState {
      return when (request) {
        is ElevatorRequest.Home -> ElevatorState.HOME
        is ElevatorRequest.OpenLoop -> ElevatorState.OPEN_LOOP
        is ElevatorRequest.TargetingPosition -> ElevatorState.TARGETING_POSITION
      }
    }

    inline fun fromHeightToPosition(height: Length): Length {
      return height / ElevatorConstants.ELEVATOR_ANGLE.sin
    }

    inline fun fromPositionToHeight(position: Length): Length {
      return position * ElevatorConstants.ELEVATOR_ANGLE.sin
    }

    inline fun fromRangeToPosition(range: Length): Length {
      return range / ElevatorConstants.ELEVATOR_ANGLE.cos
    }

    inline fun fromPositionToRange(position: Length): Length {
      return position * ElevatorConstants.ELEVATOR_ANGLE.cos
    }
  }
}
