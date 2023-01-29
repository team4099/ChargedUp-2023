package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class Intake(val io: IntakeIO) : SubsystemBase() {

  val inputs = IntakeIO.IntakeIOInputs()

  var armFeedForward =
    ArmFeedforward(
      IntakeConstants.NEO_ARM_KS,
      IntakeConstants.ARM_KG,
      IntakeConstants.ARM_KV,
      IntakeConstants.ARM_KA
    )

  private val kP =
    LoggedTunableValue("GroundIntake/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "GroundIntake/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "GroundIntake/kd",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private val rollerKV =
    LoggedTunableValue(
      "GRoundIntake/rollerKV",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )

  val forwardLimitReached: Boolean
    get() = inputs.armPosition >= IntakeConstants.ARM_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.armPosition <= IntakeConstants.ARM_MIN_ROTATION

  var lastIntakeRunTime = Clock.fpgaTime

  val rollerState: IntakeConstants.rollerStates
    get() {
      for (state in IntakeConstants.rollerStates.values()) {
        if ((state.velocity - inputs.armVelocity).absoluteValue <=
          IntakeConstants.ROLLERR_TOLERANCE
        ) {
          return state
        }
      }
      return IntakeConstants.rollerStates.DUMMY
    }

  /*
  set(state) {
    io.setRollerPower(state.power)
    if (state == IntakeConstants.ROLLER_STATE.INTAKE) {
      lastIntakeRunTime = Clock.fpgaTime
    }
    field = state
  }
   */

  val armState: IntakeConstants.armStates
    get() {
      for (state in IntakeConstants.armStates.values()) {
        if ((state.position - inputs.armPosition).absoluteValue <= IntakeConstants.ARM_TOLERANCE) {
          return state
        }
      }
      return IntakeConstants.armStates.DUMMY
    }

  var armConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      IntakeConstants.MAX_ARM_VELOCITY, IntakeConstants.MAX_ARM_ACCELERATION
    )

  var prevArmSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(IntakeConstants.NEO_KP)
      kI.initDefault(IntakeConstants.NEO_KI)
      kD.initDefault(IntakeConstants.NEO_KD)
      rollerKV.initDefault(IntakeConstants.NEO_ROLLER_KV)

      armFeedForward =
        ArmFeedforward(
          IntakeConstants.NEO_ARM_KS,
          IntakeConstants.ARM_KG,
          IntakeConstants.ARM_KV,
          IntakeConstants.ARM_KA
        )
    } else {
      kP.initDefault(IntakeConstants.SIM_KP)
      kI.initDefault(IntakeConstants.SIM_KI)
      kD.initDefault(IntakeConstants.SIM_KD)
      rollerKV.initDefault(IntakeConstants.SIM_ROLLER_KV)

      var armFeedForward =
        ArmFeedforward(
          IntakeConstants.SIM_ARM_KS,
          IntakeConstants.ARM_KG,
          IntakeConstants.ARM_KV,
          IntakeConstants.ARM_KA
        )
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    Logger.getInstance().processInputs("Intake", inputs)
  }

  fun setRollerPower(rpm: AngularVelocity) {
    val voltage = (rpm * rollerKV.get())

    io.setRollerPower(voltage)
    Logger.getInstance().recordOutput("Intake/rollerTargetSpeedRPM", rpm.inRotationsPerMinute)
    Logger.getInstance().recordOutput("Intake/RollerTargetVotlage", voltage.inVolts)
  }

  fun setRollerBrakeMode(brake: Boolean) {
    io.setRollerBrakeMode(brake)
    Logger.getInstance().recordOutput("Intake/rollerBrakeModeEnabled", brake)
  }

  fun setArmBrakeMode(brake: Boolean) {
    io.setArmBrakeMode(brake)
    Logger.getInstance().recordOutput("Intake/armBrakeModeEnabled", brake)
  }

  fun holdArmPosition(): Command {
    return run { io.setArmVoltage(armFeedForward.calculate(0.degrees, 0.degrees.perSecond)) }
  }

  fun setArmPosition(setPoint: TrapezoidProfile.State<Radian>) {
    val armAngularAcceleration =
      (setPoint.velocity - prevArmSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevArmSetpoint = setPoint

    val feedforward =
      armFeedForward.calculate(setPoint.position, setPoint.velocity, armAngularAcceleration)

    if ((forwardLimitReached && setPoint.velocity > 0.degrees.perSecond) ||
      (reverseLimitReached && setPoint.velocity < 0.degrees.perSecond)
    ) {
      io.setArmVoltage(armFeedForward.calculate(0.degrees, 0.degrees.perSecond))
    } else {
      io.setArmPosition(setPoint.position, feedforward)
    }
  }

  fun rotateArmPosition(position: Angle): Command {
    var armProfile =
      TrapezoidProfile(
        armConstraints,
        TrapezoidProfile.State(position, 0.degrees.perSecond),
        TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
      )

    var startTime = Clock.fpgaTime

    return run {
      setArmPosition(armProfile.calculate(Clock.fpgaTime - startTime))
      Logger.getInstance()
        .recordOutput(
          "Intake/isAtSetPoint",
          (position - inputs.armPosition).absoluteValue <= IntakeConstants.ARM_TOLERANCE
        )
    }
      .beforeStarting(
        {
          startTime = Clock.fpgaTime
          Logger.getInstance().recordOutput("Intake/isAtSetPoint", false)
          armProfile =
            TrapezoidProfile(
              armConstraints,
              TrapezoidProfile.State(position, 0.degrees.perSecond),
              TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)
            )
        },
        this
      )
      .until { armProfile.isFinished(Clock.fpgaTime - startTime) }
  }
}
