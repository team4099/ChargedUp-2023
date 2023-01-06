package com.team4099.robot2023.util

import com.team4099.robot2023.config.constants.Constants
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard

class Alert(group: String, text: String = "1337", type: AlertType) {
  private val type: AlertType
  private var active = false
  private var activeStartTime = 0.0
  private lateinit var text: String

  /**
   * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  constructor(text: String = "1337", type: AlertType) : this("Alerts", text, type) {}

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also
   * be sent to the console.
   */
  fun set(active: Boolean) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp()
      when (type) {
        AlertType.ERROR -> DriverStation.reportError(text, false)
        AlertType.WARNING -> DriverStation.reportWarning(text, false)
        AlertType.INFO -> println(text)
      }
    }
    this.active = active
  }

  /** Updates current alert text. */
  fun setText(text: String) {
    this.text = text
  }

  private class SendableAlerts : Sendable {
    val alerts: MutableList<Alert> = ArrayList()
    fun getStrings(type: AlertType): List<String> {
      val timeSorter = Comparator { a1: Alert, a2: Alert ->
        (a2.activeStartTime - a1.activeStartTime).toInt()
      }

      return alerts
        .filter { x: Alert -> x.type == type && x.active }
        .sortedBy { it.activeStartTime }
        .map { a: Alert -> a.text }
    }

    override fun initSendable(builder: SendableBuilder) {
      builder.setSmartDashboardType("Alerts")
      builder.addStringArrayProperty("errors", { getStrings(AlertType.ERROR).toTypedArray() }, null)
      builder.addStringArrayProperty(
        "warnings", { getStrings(AlertType.WARNING).toTypedArray() }, null
      )
      builder.addStringArrayProperty("infos", { getStrings(AlertType.INFO).toTypedArray() }, null)
    }
  }

  /** Represents an alert's level of urgency. */
  enum class AlertType {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
     * for problems which will seriously affect the robot's functionality and thus require immediate
     * attention.
     */
    ERROR,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
     * type for problems which could affect the robot's functionality but do not necessarily require
     * immediate attention.
     */
    WARNING,

    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
     * for problems which are unlikely to affect the robot's functionality, or any other alerts
     * which do not fall under "ERROR" or "WARNING".
     */
    INFO
  }

  companion object {
    private val groups: MutableMap<String, SendableAlerts> = HashMap()
  }

  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
   *
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, also used as NetworkTables title
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  init {

    if (!groups.containsKey(group)) {
      groups[group] = SendableAlerts()
      for (tabName in Constants.Alert.TABS) {
        Shuffleboard.getTab(tabName).add(group, groups[group]).withSize(2, 2).withPosition(0, 0)
      }
    }
    this.text = text
    this.type = type
    groups[group]?.alerts?.add(this)
  }
}
