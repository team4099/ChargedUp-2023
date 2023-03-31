package com.team4099.robot2023.subsystems.falconspin

import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.inVolts

object MotorChecker {

  val subsystemHardware = HashMap<String, HashMap<String, MutableList<MotorCollection>>>()

  fun add(subsystemName: String, subCategory: String, subsystemMotorCollections: MotorCollection) {
    if (subsystemHardware[subsystemName] == null) {
      subsystemHardware[subsystemName] = HashMap()
    }
    if (subsystemHardware[subsystemName]?.get(subCategory) == null) {
      subsystemHardware[subsystemName]?.set(subCategory, mutableListOf())
    }

    subsystemHardware[subsystemName]!![subCategory]!!.add(subsystemMotorCollections)
  }

  fun periodic() {
    for (subsystemName in subsystemHardware.keys) {

      for (subCategory in subsystemHardware[subsystemName]!!) {

        for (motorCollection in subCategory.value) {

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

            logMotor(subsystemName, motor)
          }
        }
      }
    }
  }
}

// not clean but whatever
fun logMotor(subsystemName: String, motor: Motor<MotorType>) {
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/AppliedVoltageVolts",
      motor.appliedVoltage.inVolts
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/BusVoltageVolts", motor.busVoltage.inVolts
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/TemperatureCelsius",
      motor.temperature.inCelsius
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/StatorCurrentAmps",
      motor.statorCurrent.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/SupplyCurrentAmps",
      motor.supplyCurrent.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/CurrentLimitStage",
      motor.currentLimitStage.name
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/BaseCurrentLimitAmps",
      motor.baseCurrentLimit.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/FirstStageTemperatureLimitCelsius",
      motor.firstStageTemperatureLimit.inCelsius
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/FirstStageCurrentLimitAmps",
      motor.firstStageCurrentLimit.inAmperes
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/MotorShutDownThresholdCelsius",
      motor.motorShutDownThreshold.inCelsius
    )
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/CurrentLimitInUseAmps",
      motor.currentLimitInUse.inAmperes
    )
  Logger.getInstance()
    .recordOutput("MotorChecker/$subsystemName/${motor.name}/MotorID", motor.id.toLong())
  Logger.getInstance()
    .recordOutput("MotorChecker/$subsystemName/${motor.name}/Errors", motor.errors.toTypedArray())
  Logger.getInstance()
    .recordOutput(
      "MotorChecker/$subsystemName/${motor.name}/Warnings", motor.warnings.toTypedArray()
    )
  Logger.getInstance()
    .recordOutput("MotorChecker/$subsystemName/${motor.name}/Info", motor.info.toTypedArray())
}
