package com.team4099.lib.logging

import com.team4099.robot2022.config.constants.Constants
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * Taken from
 * https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/util/TunableNumber.java
 */
class TunableNumber(dashboardKey: String, val defaultValue: Double) {
  private val tableKey = "TunableNumbers"
  private val key: String
  private var lastHasChangedValue = defaultValue

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  init {
    key = tableKey + "/" + dashboardKey
    if (Constants.Tuning.TUNING_MODE) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue))
    } else {
      SmartDashboard.delete(key)
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  val value: Double
    get() {
      return if (Constants.Tuning.TUNING_MODE) SmartDashboard.getNumber(key, defaultValue)
      else defaultValue
    }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   * otherwise
   */
  fun hasChanged(): Boolean {
    val currentValue = value
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue
      return true
    }
    return false
  }
}
