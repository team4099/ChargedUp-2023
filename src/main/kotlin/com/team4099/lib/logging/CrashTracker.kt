package com.team4099.lib.logging

import java.io.BufferedWriter
import java.io.FileWriter
import java.io.IOException
import java.io.PrintWriter
import java.util.Date
import java.util.UUID

/** Tracks startup and caught crash events, logging them to a file. */
object CrashTracker {
  private val RUN_INSTANCE_UUID = UUID.randomUUID()

  /**
   * Log a crash.
   *
   * @param function A human readable name for the location where the crash occurred.
   * @param throwable The caught throwable.
   */
  fun logThrowableCrash(function: String, throwable: Throwable) {
    logMarker("Exception @ $function, ${throwable.message}", throwable)
  }

  /**
   * Log an event with no associated throwable.
   *
   * @param mark The text to log.
   */
  fun logMarker(mark: String) {
    logMarker(mark, null)
  }

  /**
   * Write an event and associated throwable to a file.
   *
   * @param mark The text to log.
   * @param throwable The caught throwable.
   */
  private fun logMarker(mark: String, throwable: Throwable?) {
    //    Logger.addEvent("CRASH", mark)
    try {
      FileWriter("/home/lvuser/crash_tracking.txt", true).use { fw ->
        BufferedWriter(fw).use { bw ->
          PrintWriter(bw).use { out ->
            out.print(RUN_INSTANCE_UUID.toString())
            out.print(", ")
            out.print(mark)
            out.print(", ")
            out.print(Date().toString())

            if (throwable != null) {
              out.print(", ")
              throwable.printStackTrace(out)
            }

            out.println()
          }
        }
      }
    } catch (ex: IOException) {
      ex.printStackTrace()
    }
  }
}
