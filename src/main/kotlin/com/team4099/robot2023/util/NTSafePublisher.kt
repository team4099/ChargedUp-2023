package com.team4099.robot2023.util

import edu.wpi.first.networktables.GenericPublisher
import edu.wpi.first.networktables.IntegerPublisher
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.LogDataReceiver
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.LogTable.LogValue
import org.littletonrobotics.junction.LogTable.LoggableType

class NTSafePublisher : LogDataReceiver {
  private val rootTable: NetworkTable = NetworkTableInstance.getDefault().getTable("/AdvantageKit")
  private var lastTable = LogTable(0)
  private val timestampPublisher: IntegerPublisher =
    rootTable
      .getIntegerTopic(LogDataReceiver.timestampKey.substring(1))
      .publish(PubSubOption.sendAll(true))
  private val publishers: MutableMap<String, GenericPublisher?> = HashMap()

  override fun putTable(table: LogTable) {
    // only stream data if we are not connected to FMS to reduce bandwidth issues
    if (!DriverStation.isFMSAttached()) {
      // Send timestamp
      timestampPublisher[table.timestamp] = table.timestamp

      // Get old and new data
      val newMap = table.getAll(false)
      val oldMap = lastTable.getAll(false)

      // Encode new/changed fields
      for (field: Map.Entry<String, LogValue> in newMap) {
        // Check if field has changed
        val newValue = field.value
        if (newValue == oldMap[field.key]) {
          continue
        }

        // Create publisher if necessary
        val key = field.key.substring(1)
        var publisher = publishers[key]
        if (publisher == null) {
          publisher =
            rootTable
              .getTopic(key)
              .genericPublish(newValue.type.nT4Type, PubSubOption.sendAll(true))
          publishers[key] = publisher
        }

        when (newValue.type) {
          LoggableType.Raw -> publisher!!.setRaw(newValue.raw, table.timestamp)
          LoggableType.Boolean -> publisher!!.setBoolean(newValue.boolean, table.timestamp)
          LoggableType.BooleanArray ->
            publisher!!.setBooleanArray(newValue.booleanArray, table.timestamp)
          LoggableType.Integer -> publisher!!.setInteger(newValue.integer, table.timestamp)
          LoggableType.IntegerArray ->
            publisher!!.setIntegerArray(newValue.integerArray, table.timestamp)
          LoggableType.Float -> publisher!!.setFloat(newValue.float, table.timestamp)
          LoggableType.FloatArray -> publisher!!.setFloatArray(newValue.floatArray, table.timestamp)
          LoggableType.Double -> publisher!!.setDouble(newValue.double, table.timestamp)
          LoggableType.DoubleArray ->
            publisher!!.setDoubleArray(newValue.doubleArray, table.timestamp)
          LoggableType.String -> publisher!!.setString(newValue.string, table.timestamp)
          LoggableType.StringArray ->
            publisher!!.setStringArray(newValue.stringArray, table.timestamp)
        }
      }

      // Update last table
      lastTable = table
    }
  }
}
