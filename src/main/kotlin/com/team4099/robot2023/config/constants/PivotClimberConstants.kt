package com.team4099.robot2023.config.constants

import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.degrees
import com.team4099.lib.units.derived.radians
import com.team4099.lib.units.perSecond
import edu.wpi.first.wpilibj.DoubleSolenoid

object PivotClimberConstants {
  const val KP = 0.0
  const val KI = 0.0
  const val KD = 0.0
  const val KFF = 0.0

  val NO_LOAD_KS = 0.0
  val NO_LOAD_KV = 0.0
  val NO_LOAD_KA = 0.0

  val LOAD_KS = 0.0
  val LOAD_KV = 0.0
  val LOAD_KA = 0.0

  val MAX_VELOCITY = 0.0.radians.perSecond
  val MAX_ACCELERATION = 0.0.radians.perSecond.perSecond

  val RETRACT_TIME = 0.5.seconds
  val EXTEND_PIVOT_TIME = 0.2.seconds

  enum class PivotArmPosition(val angle: Angle) {
    OUT(0.degrees),
    IN(0.degrees)
  }

  enum class ActualPivotStates(val correspondingDesiredState: DesiredPivotStates) {
    IN(DesiredPivotStates.IN),
    BETWEEN_IN_AND_OUT(DesiredPivotStates.DUMMY),
    OUT(DesiredPivotStates.OUT)
  }

  enum class DesiredPivotStates(val extendPosition: DoubleSolenoid.Value) {
    IN(DoubleSolenoid.Value.kReverse),
    OUT(DoubleSolenoid.Value.kForward),
    DUMMY(DoubleSolenoid.Value.kOff)
  }

  const val GEAR_RATIO = (15.0 / 36.0) * (1.0 / 125.0)

  const val SMART_CURRENT_LIMIT = 40

  const val TAB = "Pivot Climber"

  val ALLOWED_ANGLE_ERROR = 1.0.degrees
}
