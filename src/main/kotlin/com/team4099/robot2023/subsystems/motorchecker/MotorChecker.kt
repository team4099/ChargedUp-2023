package com.team4099.robot2023.subsystems.motorchecker

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.inVolts

object MotorChecker : SubsystemBase() {

  val motors = mutableListOf<MotorCollection>()

  fun add(subsystemMotorCollection: MotorCollection) {
    motors.add(subsystemMotorCollection)
  }

  override fun periodic() {
    for (motorCollection in motors) {

      // base current limit
      if (motorCollection.maxMotorTemperature < motorCollection.firstStageTemperatureLimit &&
        motorCollection.currentLimitStage != CURRENT_STAGE_LIMIT.BASE
      ) {
        motorCollection.setCurrentLimit(motorCollection.baseCurrentLimit)
      }

      // first stage current limit
      if (motorCollection.maxMotorTemperature in
        motorCollection.firstStageTemperatureLimit..motorCollection.motorShutDownThreshold &&
        motorCollection.currentLimitStage != CURRENT_STAGE_LIMIT.FIRST
      ) {
        motorCollection.setCurrentLimit(motorCollection.firstStageCurrentLimit)
      }

      for (motor in motorCollection.motorCollection) {
        // complete motor shutdown but we don't want to shut down all motors at once
        if (motor.temperature > motor.motorShutDownThreshold) {
          motor.shutdown()
        }

        logMotor(motor)
      }
    }
  }
}

// not clean but whatever
fun logMotor(motor: Motor<MotorType>) {
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/AppliedVoltageVolts", motor.appliedVoltage.inVolts)
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/BusVoltageVolts", motor.busVoltage.inVolts)
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/TemperatureCelsius", motor.temperature.inCelsius)
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/StatorCurrentAmps", motor.statorCurrent.inAmperes)
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/SupplyCurrentAmps", motor.supplyCurrent.inAmperes)
  Logger.getInstance()
    .recordOutput("MotorChecker/${motor.name}/CurrentLimitStage", motor.currentLimitStage.name)
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/${motor.name}/BaseCurrentLimitAmps", motor.baseCurrentLimit.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/${motor.name}/FirstStageTemperatureLimitCelsius",
      motor.firstStageTemperatureLimit.inCelsius
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/${motor.name}/FirstStageCurrentLimitAmps",
      motor.firstStageCurrentLimit.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/${motor.name}/MotorShutDownThresholdCelsius",
      motor.motorShutDownThreshold.inCelsius
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/${motor.name}/CurrentLimitInUseAmps", motor.currentLimitInUse.inAmperes
    )
  Logger.getInstance().recordOutput("MotorChecker/${motor.name}/MotorID", motor.id.toLong())
}
