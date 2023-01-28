package com.team4099.lib.logging

import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * Taken from httpgit
 * s://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/TunableNumber.java
 */
class TunableNumber(dashboardKey: String) {
  private val key: String
  private var defaultValue = 0.0
  private var lastHasChangedValue = defaultValue

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  constructor(dashboardKey: String, defaultValue: Double) : this(dashboardKey) {
    setDefault(defaultValue)
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  fun getDefault(): Double {
    return defaultValue
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  fun setDefault(defaultValue: Double) {
    this.defaultValue = defaultValue

    // This makes sure the data is on NetworkTables but will not change it
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue))
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  fun get(): Double {
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
    val currentValue = get()
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue
      return true
    }
    return false
  }

  companion object {
    private const val tableKey = "TunableNumbers"
  }

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  init {
    key = tableKey + "/" + dashboardKey
  }
}
