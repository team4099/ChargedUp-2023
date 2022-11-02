package com.team4099.lib.logging

/**
 * A runnable which catches exceptions and reports them to [CrashTracker].
 *
 * @param name A human readable name to include with reported crashes.
 */
abstract class CrashTrackingRunnable(private val name: String) : Runnable {
  override fun run() {
    try {
      runCrashTracked()
    } catch (t: Throwable) {
      CrashTracker.logThrowableCrash("CrTrRu.run: $name", t)
      throw t
    }
  }

  /** The function which will be run by [run] with crash tracking. */
  abstract fun runCrashTracked()
}
