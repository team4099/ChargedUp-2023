package com.team4099.robot2023.subsystems.groundintake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import java.util.function.DoubleSupplier

class GroundIntake(val io: GroundIntakeIO) : SubsystemBase() {

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

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= GroundIntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= GroundIntakeConstants.ARM_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  val rollerState: GroundIntakeConstants.RollerStates
    get() = GroundIntakeConstants.RollerStates.fromVoltageToRollerState(inputs.rollerAppliedVoltage)

  val actualArmStates = hashMapOf<GroundIntakeConstants.ArmStates, LoggedTunableValue<Radian>>()

  val armState: GroundIntakeConstants.ArmStates
    get() = GroundIntakeConstants.ArmStates.fromDegreesToArmState(inputs.armPosition)

  var armConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      GroundIntakeConstants.MAX_ARM_VELOCITY, GroundIntakeConstants.MAX_ARM_ACCELERATION
    )

  var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  var positionToHold = inputs.armPosition

  var desiredPosition = 0.0.degrees

  init {
    GroundIntakeConstants.ArmStates.values().forEach {
      actualArmStates[it] =
        LoggedTunableValue<Radian>(
          "GroundIntake/${it.name}", it.position, Pair({ it.inDegrees }, { it.degrees })
        )
    }
    if (RobotBase.isReal()) {
      kP.initDefault(GroundIntakeConstants.PID.NEO_KP)
      kI.initDefault(GroundIntakeConstants.PID.NEO_KI)
      kD.initDefault(GroundIntakeConstants.PID.NEO_KD)

      armFeedforward =
        ArmFeedforward(
          GroundIntakeConstants.PID.NEO_ARM_KS,
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

    Logger.getInstance().recordOutput("GroundIntake/positionToHold", positionToHold.inDegrees)

    Logger.getInstance()
      .recordOutput(
        "GroundIntake/isAtSetpoint",
        (desiredPosition - inputs.armPosition).absoluteValue <=
          GroundIntakeConstants.ARM_TOLERANCE
      )

    Logger.getInstance().recordOutput("GroundIntake/ArmState", armState.name)

    Logger.getInstance().recordOutput("GroundIntake/RollerState", rollerState.name)
  }

  /** @param appliedVoltage Represents the applied voltage of the roller motor */
  fun setRollerVoltage(appliedVoltage: ElectricalPotential) {
    io.setRollerPower(appliedVoltage)
  }

  /**
   * Sets the break/idle mode of the arm
   *
   * @param brake The value that break mode for the arm will be set as
   */
  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
    Logger.getInstance().recordOutput("GroundIntake/groundIntakeArmBrakeModeEnabled", brake)
  }

  fun groundIntakeExtendCommand(): Command {
    var desiredAngle = GroundIntakeConstants.ArmStates.INTAKE.position
    val supplier = DoubleSupplier { desiredAngle.inDegrees }

    val setupCommand = runOnce {
      if (Constants.Tuning.TUNING_MODE) {
        desiredAngle =
          actualArmStates[GroundIntakeConstants.ArmStates.INTAKE]?.get()
            ?: GroundIntakeConstants.ArmStates.DUMMY.position
      }
    }

    val extendCommand = rotateGroundIntakeToAngle(supplier).andThen(holdArmPosition())

    val returnCommand = setupCommand.andThen(extendCommand)

    // TODO: change name
    returnCommand.name = "GroundIntakeExtendCommand"
    return returnCommand
  }

  fun openLoopCommand(voltage: ElectricalPotential): Command {
    val returnCommand =
      run { io.setArmVoltage(voltage) }.unless {
        forwardLimitReached && voltage > 0.0.volts || reverseLimitReached && voltage < 0.0.volts
      }

    returnCommand.name = "GroundIntakeOpenLoopCommand"
    return returnCommand
  }

  /** Tells the feedforward not to move the arm */
  fun holdArmPosition(toAngle: Angle = positionToHold): Command {
    var holdPosition = toAngle

    val holdPositionCommand =
      run {
        io.setArmPosition(
          holdPosition, armFeedforward.calculate(0.degrees, 0.degrees.perSecond)
        )
      }
        .beforeStarting({ holdPosition = positionToHold }, this)

    holdPositionCommand.name = "GroundIntakeHoldPositionCommand"
    return holdPositionCommand
  }

  /**
   * Sets the arm position using the trapezoidal profile state
   *
   * @param setpoint.position Represents the position the arm should go to
   * @param setpoint.velocity Represents the velocity the arm should be at
   */
  fun setArmPosition(setpoint: TrapezoidProfile.State<Radian>) {

    // Calculating the acceleration of the arm
    val armAngularAcceleration =
      (setpoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setpoint

    // Set up the feed forward variable
    val feedforward =
      armFeedforward.calculate(setpoint.position, setpoint.velocity, armAngularAcceleration)

    // set the desired position to be the setpoint's position
    desiredPosition = setpoint.position

    // When the forward or reverse limit is reached, set the voltage to 0
    // Else mose the arm to the setpoint position
    if ((forwardLimitReached && setpoint.velocity > 0.degrees.perSecond) ||
      (reverseLimitReached && setpoint.velocity < 0.degrees.perSecond)
    ) {
      io.setArmVoltage(armFeedforward.calculate(0.degrees, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setpoint.position, feedforward)
    }

    Logger.getInstance()
      .recordOutput("GroundIntake/groundIntakeArmTargetPosition", setpoint.position.inDegrees)
  }

  /**
   * Command for creating a profile to a position and following that profile until reached
   *
   * @param angle The angle the arm should go to
   */
  fun rotateGroundIntakeToAngle(supplier: DoubleSupplier): Command {
    // Generate a trapezoidal profile from the current position to the setpoint
    // with set constraints
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(supplier.asDouble.degrees, 0.degrees.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )

    // Initializing a variable of start time which represents the start of elapsed
    // time of the profile
    var startTime = Clock.fpgaTime

    // Creates and returns a command that can be run continuously until the profile finishes
    val rotateArmPositionCommand =
      runOnce {
        // Resets the initial time since the time at the start of method is of robot init
        startTime = Clock.fpgaTime
        // Regenerates profile with the new position passed in
        armProfile =
          TrapezoidProfile(
            armConstraints,
            TrapezoidProfile.State(supplier.asDouble.degrees, 0.degrees.perSecond),
            TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
          )
      }
        .andThen(
          run {
            // Regenerates profile state for that loop cycle and sets to that position
            setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
            positionToHold = inputs.armPosition

            Logger.getInstance()
              .recordOutput(
                "GroundIntake/groundIntakeSetpoint", supplier.asDouble.degrees.inDegrees
              )
          }
            .until {
              // Run the lambda until the predicted finishing time of the profile elapses
              armProfile.isFinished(Clock.fpgaTime - startTime)
            }
        )
        .finallyDo { positionToHold = inputs.armPosition }
        .handleInterrupt { positionToHold = inputs.armPosition }

    rotateArmPositionCommand.name = "RotateGroundIntakeCommand"
    return rotateArmPositionCommand
  }
}
