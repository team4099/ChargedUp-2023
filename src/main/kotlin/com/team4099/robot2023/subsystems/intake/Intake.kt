package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegrees
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts

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
    LoggedTunableValue("GroundIntake/kP", Pair({ it.inVoltsPerDegrees }, { it.volts.perDegree }))
  private val kI =
    LoggedTunableValue(
      "GroundIntake/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val kD =
    LoggedTunableValue(
      "GroundIntake/kd",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
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

  var constraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      IntakeConstants.MAX_ARM_VELOCITY, IntakeConstants.MAX_ARM_ACCELERATION
    )

  var armSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.armPosition, inputs.armVelocity)

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(IntakeConstants.NEO_KP)
      kI.initDefault(IntakeConstants.NEO_KI)
      kD.initDefault(IntakeConstants.NEO_KD)

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
}
