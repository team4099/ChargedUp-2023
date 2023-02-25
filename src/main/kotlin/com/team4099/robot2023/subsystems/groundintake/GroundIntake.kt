package com.team4099.robot2023.subsystems.groundintake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.Request.GroundIntakeRequest as GroundIntakeRequest

class GroundIntake(private val io: GroundIntakeIO) {

  val inputs = GroundIntakeIO.GroundIntakeIOInputs()

  var armFeedforward: ArmFeedforward

  private val kP =
    LoggedTunableValue("GroundIntake/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "GroundIntake/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "GroundIntake/kD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object TunableGroundIntakeStates {
    val intakeAngle =
      LoggedTunableValue(
        "GroundIntake/intakeAngle",
        GroundIntakeConstants.INTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val outtakeAngle =
      LoggedTunableValue(
        "GroundIntake/outtakeAngle",
        GroundIntakeConstants.OUTTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val stowedUpAngle =
      LoggedTunableValue(
        "GroundIntake/stowedUpAngle",
        GroundIntakeConstants.STOWED_UP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val stowedDownAngle =
      LoggedTunableValue(
        "GroundIntake/stowedDownAngle",
        GroundIntakeConstants.STOWED_DOWN_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeVoltage =
      LoggedTunableValue(
        "GroundIntake/intakeVoltage",
        GroundIntakeConstants.INTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val outtakeVoltage =
      LoggedTunableValue(
        "GroundIntake/outtakeVoltage",
        GroundIntakeConstants.OUTTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val neutralVoltage =
      LoggedTunableValue(
        "GroundIntake/neutralVoltage",
        GroundIntakeConstants.NEUTRAL_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
  }

  var armPositionTarget: Angle = 0.0.degrees

  var armVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  var isZeroed = false

  private var lastArmPositionTarget = 0.0.degrees

  private var lastArmVoltage = 0.0.volts

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_MIN_ROTATION

  val openLoopForwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_OPEN_LOOP_MAX_ROTATION

  val openLoopReverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_OPEN_LOOP_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  var currentState: GroundIntakeState = GroundIntakeState.UNINITIALIZED

  var currentRequest: GroundIntakeRequest = GroundIntakeRequest.ZeroArm()
    set(value) {
      when (value) {
        is GroundIntakeRequest.OpenLoop -> {
          armVoltageTarget = value.voltage
          rollerVoltageTarget = value.rollerVoltage
        }
        is GroundIntakeRequest.TargetingPosition -> {
          armPositionTarget = value.position
          rollerVoltageTarget = value.rollerVoltage
        }
        else -> {}
      }
      field = value
    }

  private var armConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      GroundIntakeConstants.MAX_ARM_VELOCITY, GroundIntakeConstants.MAX_ARM_ACCELERATION
    )

  private var armProfile =
    TrapezoidProfile(
      armConstraints,
      TrapezoidProfile.State(armPositionTarget, 0.0.degrees.perSecond),
      TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
    )

  private var timeProfileGeneratedAt = Clock.fpgaTime

  private var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  val isAtTargetedPosition: Boolean
    get() =
      currentState == GroundIntakeState.TARGETING_POSITION &&
        armProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
        (inputs.armPosition - armPositionTarget).absoluteValue <=
        GroundIntakeConstants.ARM_TOLERANCE

  init {

    if (RobotBase.isReal()) {
      kP.initDefault(GroundIntakeConstants.PID.NEO_KP)
      kI.initDefault(GroundIntakeConstants.PID.NEO_KI)
      kD.initDefault(GroundIntakeConstants.PID.NEO_KD)

      armFeedforward =
        ArmFeedforward(
          GroundIntakeConstants.PID.ARM_KS,
          GroundIntakeConstants.PID.ARM_KG,
          GroundIntakeConstants.PID.ARM_KV,
          GroundIntakeConstants.PID.ARM_KA
        )
    } else {
      kP.initDefault(GroundIntakeConstants.PID.SIM_KP)
      kI.initDefault(GroundIntakeConstants.PID.SIM_KI)
      kD.initDefault(GroundIntakeConstants.PID.SIM_KD)

      armFeedforward =
        ArmFeedforward(
          0.0.volts,
          GroundIntakeConstants.PID.ARM_KG,
          GroundIntakeConstants.PID.ARM_KV,
          GroundIntakeConstants.PID.ARM_KA
        )
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("GroundIntake", inputs)

    Logger.getInstance().recordOutput("GroundIntake/currentState", currentState.name)

    Logger.getInstance()
      .recordOutput("GroundIntake/requestedState", currentRequest.javaClass.simpleName)

    Logger.getInstance().recordOutput("GroundIntake/isAtTargetedPosition", isAtTargetedPosition)

    Logger.getInstance().recordOutput("GroundIntake/isZeroed", isZeroed)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.getInstance()
        .recordOutput(
          "GroundIntake/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
        )

      Logger.getInstance()
        .recordOutput("GroundIntake/timeProfileGeneratedAt", timeProfileGeneratedAt.inSeconds)

      Logger.getInstance()
        .recordOutput("GroundIntake/armPositionTarget", armPositionTarget.inDegrees)

      Logger.getInstance().recordOutput("GroundIntake/armVoltageTarget", armVoltageTarget.inVolts)

      Logger.getInstance()
        .recordOutput("GroundIntake/rollerVoltageTarget", rollerVoltageTarget.inVolts)

      Logger.getInstance()
        .recordOutput("GroundIntake/lastCommandedAngle", lastArmPositionTarget.inDegrees)

      Logger.getInstance().recordOutput("GroundIntake/forwardLimitReached", forwardLimitReached)

      Logger.getInstance().recordOutput("GroundIntake/reverseLimitReached", reverseLimitReached)
    }

    var nextState = currentState
    when (currentState) {
      GroundIntakeState.UNINITIALIZED -> {
        // Outputs
        // No designated output functionality because targeting position will take care of it next
        // loop cycle

        // Transitions
        nextState = GroundIntakeState.ZEROING_ARM
      }
      GroundIntakeState.ZEROING_ARM -> {
        zeroArm()

        if (inputs.isSimulated ||
          (inputs.armPosition - inputs.armAbsoluteEncoderPosition).absoluteValue <= 1.degrees
        ) {
          isZeroed = true
          lastArmPositionTarget = -1337.degrees
        }

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      GroundIntakeState.OPEN_LOOP_REQUEST -> {
        // Outputs
        if (armVoltageTarget != lastArmVoltage) {
          lastIntakeRunTime = Clock.fpgaTime
        }

        setArmVoltage(armVoltageTarget)
        setRollerVoltage(rollerVoltageTarget)

        // Transitions
        nextState = fromRequestToState(currentRequest)

        // See related comment in targeting position to see why we do this
        if (!(currentState.equivalentToRequest(currentRequest))) {
          lastArmVoltage = -1337.volts
        }
      }
      GroundIntakeState.TARGETING_POSITION -> {
        // Outputs
        if (armPositionTarget != lastArmPositionTarget) {
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(armPositionTarget, 0.0.degrees.perSecond),
              TrapezoidProfile.State(inputs.armPosition, 0.0.degrees.perSecond)
            )
          timeProfileGeneratedAt = Clock.fpgaTime

          // This statement is only run when the armPositionTarget is first noticed to be different
          // than the previous setpoint the arm went to.
          lastArmPositionTarget = armPositionTarget
          lastIntakeRunTime = Clock.fpgaTime
        }

        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt

        val profileOutput = armProfile.calculate(timeElapsed)

        setArmPosition(profileOutput)
        if (armProfile.isFinished(timeElapsed)) {
          setRollerVoltage(rollerVoltageTarget)
        }

        Logger.getInstance()
          .recordOutput("GroundIntake/completedMotionProfile", armProfile.isFinished(timeElapsed))

        Logger.getInstance()
          .recordOutput("GroundIntake/profilePositionDegrees", profileOutput.position.inDegrees)
        Logger.getInstance()
          .recordOutput(
            "GroundIntake/profileVelocityDegreesPerSecond",
            profileOutput.velocity.inDegreesPerSecond
          )

        // Transitions
        nextState = fromRequestToState(currentRequest)

        // if we're transitioning out of targeting position, we want to make sure the next time we
        // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
        // the previous time we ran it)
        if (!(currentState.equivalentToRequest(currentRequest))) {
          // setting the last target to something unreasonable so the profile is generated next loop
          // cycle
          lastArmPositionTarget = (-1337).degrees
        }
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.
    currentState = nextState
  }

  /** @param appliedVoltage Represents the applied voltage of the roller motor */
  fun setRollerVoltage(appliedVoltage: ElectricalPotential) {
    io.setRollerVoltage(appliedVoltage)
  }

  /**
   * Sets the break/idle mode of the arm
   *
   * @param brake The value that break mode for the arm will be set as
   */
  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
    Logger.getInstance().recordOutput("GroundIntake/armBrakeModeEnabled", brake)
  }

  fun zeroArm() {
    io.zeroEncoder()
  }

  fun regenerateProfileNextLoopCycle() {
    lastArmVoltage = -3337.volts
    lastArmPositionTarget = -3337.degrees
    lastIntakeRunTime = -3337.seconds
  }

  fun setArmVoltage(voltage: ElectricalPotential) {
    if ((openLoopForwardLimitReached && voltage > 0.0.volts) ||
      (openLoopReverseLimitReached && voltage < 0.0.volts)
    ) {
      io.setArmVoltage(0.0.volts)
    } else {
      io.setArmVoltage(voltage)
    }
  }

  /**
   * Sets the arm position using the trapezoidal profile state
   *
   * @param setpoint.position Represents the position the arm should go to
   * @param setpoint.velocity Represents the velocity the arm should be at
   */
  private fun setArmPosition(setpoint: TrapezoidProfile.State<Radian>) {

    // Calculating the acceleration of the arm
    val armAngularAcceleration =
      (setpoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setpoint

    // Set up the feed forward variable
    val feedforward =
      armFeedforward.calculate(setpoint.position, setpoint.velocity, armAngularAcceleration)

    // When the forward or reverse limit is reached, set the voltage to 0
    // Else move the arm to the setpoint position
    if (isOutOfBounds(setpoint.velocity)) {
      io.setArmVoltage(armFeedforward.calculate(inputs.armPosition, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }

    Logger.getInstance()
      .recordOutput("GroundIntake/profileIsOutOfBounds", isOutOfBounds(setpoint.velocity))
    Logger.getInstance().recordOutput("GroundIntake/armFeedForward", feedforward.inVolts)
    Logger.getInstance().recordOutput("GroundIntake/armTargetPosition", setpoint.position.inDegrees)
    Logger.getInstance()
      .recordOutput("GroundIntake/armTargetVelocity", setpoint.velocity.inDegreesPerSecond)
  }

  private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
    return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
      (velocity < 0.0.degrees.perSecond && reverseLimitReached)
  }

  companion object {
    enum class GroundIntakeState {
      UNINITIALIZED,
      ZEROING_ARM,
      TARGETING_POSITION,
      OPEN_LOOP_REQUEST;

      inline fun equivalentToRequest(request: GroundIntakeRequest): Boolean {
        return (
          (request is GroundIntakeRequest.OpenLoop && this == OPEN_LOOP_REQUEST) ||
            (request is GroundIntakeRequest.TargetingPosition && this == TARGETING_POSITION)
          )
      }
    }

    inline fun fromRequestToState(request: GroundIntakeRequest): GroundIntakeState {
      return when (request) {
        is GroundIntakeRequest.OpenLoop -> GroundIntakeState.OPEN_LOOP_REQUEST
        is GroundIntakeRequest.TargetingPosition -> GroundIntakeState.TARGETING_POSITION
        is GroundIntakeRequest.ZeroArm -> GroundIntakeState.ZEROING_ARM
      }
    }
  }
}
