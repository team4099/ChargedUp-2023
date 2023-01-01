package com.team4099.lib.logging

import com.team4099.robot2023.config.constants.Constants
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
class LoggedTunableNumber(private val dashboardKey: String, private val defaultValue: Double) {
  private val key = "$tableKey/$dashboardKey"
  private var dashboardNumber: LoggedDashboardNumber? = null
  private var lastHasChangedValue = 0.0

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  init {
    if (Constants.Tuning.TUNING_MODE) {
      dashboardNumber = LoggedDashboardNumber(key, defaultValue)
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  fun get(): Double {
    // TODO: clean up (there are two elses here, one for the null case and one for the non tuning
    // mode case)
    return if (Constants.Tuning.TUNING_MODE) dashboardNumber?.get() ?: defaultValue
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
}
