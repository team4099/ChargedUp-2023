package com.team4099.robot2023.subsystems.groundintake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import com.team4099.robot2023.subsystems.superstructure.SuperStructureState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
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
import com.team4099.robot2023.subsystems.superstructure.SuperStructureState.GroundIntakeStructure.GroundIntakeRequest as GroundIntakeRequest
import com.team4099.robot2023.subsystems.superstructure.SuperStructureState.GroundIntakeStructure.GroundIntakeState as GroundIntakeState

class GroundIntake(private val io: GroundIntakeIO) : SubsystemBase() {

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

  private val intakeAngle =
    LoggedTunableValue(
      "GroundIntake/intakeAngle",
      GroundIntakeConstants.INTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val outtakeAngle =
    LoggedTunableValue(
      "GroundIntake/outtakeAngle",
      GroundIntakeConstants.OUTTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedUpAngle =
    LoggedTunableValue(
      "GroundIntake/stowedUpAngle",
      GroundIntakeConstants.STOWED_UP_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val stowedDownAngle =
    LoggedTunableValue(
      "GroundIntake/stowedDownAngle",
      GroundIntakeConstants.STOWED_DOWN_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  private val intakeVoltage =
    LoggedTunableValue(
      "GroundIntake/intakeVoltage",
      GroundIntakeConstants.INTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val outtakeVoltage =
    LoggedTunableValue(
      "GroundIntake/outtakeVoltage",
      GroundIntakeConstants.OUTTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  private val neutralVoltage =
    LoggedTunableValue(
      "GroundIntake/neutralVoltage",
      GroundIntakeConstants.NEUTRAL_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  var armPositionTarget: Angle = 0.0.degrees

  var armVoltageTarget: ElectricalPotential = 0.0.volts

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts

  private var lastArmPositionTarget = 0.0.degrees

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_MIN_ROTATION

  val openLoopForwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_OPEN_LOOP_MAX_ROTATION

  val openLoopReverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_OPEN_LOOP_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  var currentState: GroundIntakeState = GroundIntakeState.Uninitialized()

  var requestedState =
    SuperStructureState.GroundIntakeStructure.GroundIntakeRequest.TargetingPosition(
      stowedUpAngle.get()
    )

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

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("GroundIntake", inputs)

    Logger.getInstance().recordOutput("GroundIntake/currentState", currentState.javaClass.name)

    Logger.getInstance().recordOutput("GroundIntake/requestedState", requestedState.javaClass.name)

    Logger.getInstance()
      .recordOutput("GroundIntake/isAtCommandedState", requestedState.equals(currentState))

    Logger.getInstance().recordOutput("GroundIntake/armPositionTarget", armPositionTarget.inDegrees)

    Logger.getInstance().recordOutput("GroundIntake/armVoltageTarget", armVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("GroundIntake/rollerVoltageTarget", rollerVoltageTarget.inVolts)

    Logger.getInstance()
      .recordOutput("GroundIntake/lastCommandedAngle", lastArmPositionTarget.inDegrees)

    Logger.getInstance().recordOutput("GroundIntake/forwardLimitReached", forwardLimitReached)

    Logger.getInstance().recordOutput("GroundIntake/reverseLimitReached", reverseLimitReached)

    var nextState: SuperStructureState.GroundIntakeStructure = currentState
    when (currentState) {
      GroundIntakeState.Uninitialized() -> {
        // Outputs
        // No designated output functionality because targeting position will take care of it next
        // loop cycle

        // Transitions
        nextState = requestedState
      }
      GroundIntakeState.OpenLoop() -> {
        // Outputs
        setArmVoltage(armVoltageTarget)
        setRollerVoltage(rollerVoltageTarget)

        // Transitions
        nextState = requestedState
      }
      GroundIntakeState.TargetingPosition() -> {
        // Outputs
        if (armPositionTarget != lastArmPositionTarget) {
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(armPositionTarget, 0.0.degrees.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
          timeProfileGeneratedAt = Clock.fpgaTime
          lastArmPositionTarget = armPositionTarget
        }

        setArmPosition(armProfile.calculate(Clock.fpgaTime - timeProfileGeneratedAt))
        setRollerVoltage(rollerVoltageTarget)

        // Transitions
        nextState = requestedState

        // if we're transitioning out of targeting position, we want to make sure the next time we
        // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
        // the previous time we ran it)
        if (!(requestedState.equals(currentState))) {
          // setting the last target to something unreasonable so the profile is generated next loop
          // cycle
          lastArmPositionTarget = -1337.degrees
        }
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.

    when (nextState) {
      is GroundIntakeRequest.TargetingPosition ->
        currentState = GroundIntakeState.TargetingPosition()
      is GroundIntakeRequest.OpenLoop -> currentState = GroundIntakeState.OpenLoop()
    }
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

  fun intakeCommand(): CommandBase {
    val returnCommand = runOnce {
      requestedState = GroundIntakeRequest.TargetingPosition(intakeAngle.get())
      armPositionTarget = intakeAngle.get()
      rollerVoltageTarget = intakeVoltage.get()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun outtakeCommand(): CommandBase {
    val returnCommand = runOnce {
      requestedState = GroundIntakeRequest.TargetingPosition(outtakeAngle.get())
      armPositionTarget = outtakeAngle.get()
      rollerVoltageTarget = outtakeVoltage.get()
    }

    returnCommand.name = "GroundIntakeOuttakeCommand"
    return returnCommand
  }

  fun stowedUpCommand(): CommandBase {
    val returnCommand = runOnce {
      requestedState = GroundIntakeRequest.TargetingPosition(stowedUpAngle.get())
      armPositionTarget = stowedUpAngle.get()
      rollerVoltageTarget = neutralVoltage.get()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
  }

  fun stowedDownCommand(): CommandBase {
    val returnCommand = runOnce {
      requestedState = GroundIntakeRequest.TargetingPosition(stowedDownAngle.get())
      armPositionTarget = stowedDownAngle.get()
      rollerVoltageTarget = neutralVoltage.get()
    }

    returnCommand.name = "GroundIntakeIntakeCommand"
    return returnCommand
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
      io.setArmVoltage(armFeedforward.calculate(0.0.degrees, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }

    Logger.getInstance().recordOutput("GroundIntake/armTargetPosition", setpoint.position.inDegrees)
    Logger.getInstance()
      .recordOutput("GroundIntake/armTargetVelocity", setpoint.velocity.inDegreesPerSecond)
  }

  private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
    return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
      (velocity < 0.0.degrees.perSecond && reverseLimitReached)
  }
}
