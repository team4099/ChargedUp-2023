package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.inVoltsPerDegrees
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.radiansPerSecondPerRadiansPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

class Intake(val io: IntakeIO) : SubsystemBase() {

  val inputs = IntakeIO.IntakeIOInputs()
  private val kP = LoggedTunableValue("GroundIntake/kP", Pair({it.inVoltsPerRadian}, {it.volts.perRadian}))
  private val kI = LoggedTunableValue("GroundIntake/kI", Pair({it.inVoltsPerRadianSeconds}, {it.volts.perRadianSeconds}))
  private val kD = LoggedTunableValue("GroundIntake/kd", Pair({it.inVoltsPerRadianPerSecond}, {it.volts/1.0.radians.perSecond}))
  val forwardLimitReached: Boolean
    get() = inputs.leaderArmPosition >= IntakeConstants.INTAKE_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.leaderArmPosition <= IntakeConstants.INTAKE_MIN_ROTATION
  var constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(IntakeConstants.MAX_ARM_VELOCITY.inDegreesPerSecond, IntakeConstants.MAX_ARM_ACCELERATION.inDegreesPerSecondPerSecond)
  

  var lastIntakeRunTime = Clock.fpgaTime

  var rollerState = IntakeConstants.ROLLER_STATE.IDLE
    set(state) {
      io.setRollerPower(state.power)
      if (state == IntakeConstants.ROLLER_STATE.INTAKE) {
        lastIntakeRunTime = Clock.fpgaTime
      }
      field = state
    }

  init {}

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.getInstance().processInputs("Intake", inputs)
  }
}
