package com.team4099.robot2022.util

import edu.wpi.first.wpilibj.SerialPort

object BatteryTracker {
  private const val nameLength: Int = 12
  private var name: String = "BAT-00000000"
  private val scanCommand =
    byteArrayOf(0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xab.toByte(), 0xcd.toByte())
  private val responsePrefix = byteArrayOf(0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31)
  private val fullResponseLength = responsePrefix.size + nameLength + 1

  fun scanBattery(timeout: Double): String {
    try {
      val port: SerialPort = SerialPort(9600, SerialPort.Port.kUSB2)

      port.setTimeout(timeout)
      port.setWriteBufferSize(scanCommand.size)
      port.setReadBufferSize(fullResponseLength)

      port.write(scanCommand, scanCommand.size)
      val response: ByteArray = port.read(fullResponseLength)

      if (response.size != fullResponseLength) {
        println(
          "[BatteryTracker] Expected " +
            fullResponseLength +
            " bytes from scanner, got " +
            response.size
        )
        return name
      }

      for (i in responsePrefix.indices) {
        if (response[i] != responsePrefix[i]) {
          println("[BatteryTracker] Invalid prefix from scanner.  Got data:")
          println("[BatteryTracker] " + response.contentToString())
          return name
        }
      }
      // Read name from data
      val batteryNameBytes: ByteArray = ByteArray(nameLength)
      System.arraycopy(response, responsePrefix.size, batteryNameBytes, 0, nameLength)
      name = String(batteryNameBytes).substring(7, name.length)
      println("[BatteryTracker] Scanned battery " + name)
    } catch (e: Exception) {
      println("[BatteryTracker] Exception while trying to scan battery")
      e.printStackTrace()
    }

    return name
  }
}
