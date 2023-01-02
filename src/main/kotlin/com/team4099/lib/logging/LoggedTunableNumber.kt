package com.team4099.lib.logging

import com.team4099.robot2023.config.constants.Constants
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
class LoggedTunableNumber(dashboardKey: String) {
  private val key: String
  private var hasDefault = false
  private var defaultValue = 0.0
  private var dashboardNumber: LoggedDashboardNumber? = null
  private var lastHasChangedValue = 0.0

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  constructor(dashboardKey: String, defaultValue: Double) : this(dashboardKey) {
    initDefault(defaultValue)
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  fun initDefault(defaultValue: Double) {
    if (!hasDefault) {
      hasDefault = true
      this.defaultValue = defaultValue
      if (Constants.Tuning.TUNING_MODE) {
        dashboardNumber = LoggedDashboardNumber(key, defaultValue)
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  fun get(): Double {
    if (!hasDefault) {
      return 0.0
    } else {
      if (Constants.Tuning.TUNING_MODE) {
        return dashboardNumber?.get() ?: defaultValue
      } else {
        return defaultValue
      }
    }
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
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  init {
    key = tableKey + "/" + dashboardKey
  }
}
