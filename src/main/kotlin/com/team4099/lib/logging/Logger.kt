package com.team4099.lib.logging

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget
import java.io.File
import java.io.IOException
import java.lang.ClassCastException
import java.lang.IllegalArgumentException
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import java.nio.file.StandardOpenOption
import java.time.Instant

/**
 * The global logger object.
 *
 * Manages Shuffleboard entries and CSV logging of values.
 */
object Logger {
  private val dataSources = mutableListOf<LogSource<Any>>()
  private val logNotifier = Notifier(this::saveEvents)
  private lateinit var file: Path
  private lateinit var eventsFile: Path
  private var loggingLocation: String =
    when {
      RobotBase.isSimulation() -> "./logs"
      File("/media/sda1/").exists() -> "/media/sda1/logs/"
      else -> "/home/lvuser/logs/"
    }

  private var values: String = ""
    get() {
      field = dataSources.joinToString(",") { it.supplier().toString() }
      return field
    }
  private val events = mutableListOf<String>()

  /** Severity of an event. */
  enum class Severity {
    DEBUG,
    INFO,
    WARN,
    ERROR
  }

  @Throws(IOException::class)
  private fun createLogDirectory() {
    val logDirectory = File(loggingLocation)
    if (!logDirectory.exists()) {
      Files.createDirectory(Paths.get(loggingLocation))
    }
  }

  private fun createFile() {
    try {
      createLogDirectory()

      file =
        if (DriverStation.isFMSAttached()) {
          Paths.get(
            "$loggingLocation${DriverStation.getEventName()}_" +
              "${DriverStation.getMatchType()}" +
              "${DriverStation.getMatchNumber()}.csv"
          )
        } else {
          Paths.get("${loggingLocation}test.csv")
        }
      eventsFile =
        if (DriverStation.isFMSAttached()) {
          Paths.get(
            "$loggingLocation${DriverStation.getEventName()}_" +
              "${DriverStation.getMatchType()}" +
              "${DriverStation.getMatchNumber()}Events.csv"
          )
        } else {
          Paths.get("${loggingLocation}testEvents.csv")
        }

      if (Files.exists(file)) Files.delete(file)
      Files.createFile(file)

      if (Files.exists(eventsFile)) Files.delete(eventsFile)
      Files.createFile(eventsFile)

      saveTitles()
    } catch (e: Exception) {
      e.printStackTrace()
    }
  }

  /**
   * Add a source of data for the logger.
   *
   * @param tab The name of the Shuffleboard tab to add this value to. Typically the subsystem name.
   * @param name The name of this value.
   * @param supplier A function which returns the value to be logged.
   * @param setter An optional function which will be called when the value in Shuffleboard is
   * changed.
   * @param followSupplier If false, the supplier will not be passed through to the Shuffleboard
   * value.
   */
  fun <T : Any> addSource(
    tab: String,
    name: String,
    supplier: () -> T,
    setter: ((T) -> Unit)?,
    followSupplier: Boolean = true
  ) {
    var shuffleboardEntry: SimpleWidget? = null
    try {
      shuffleboardEntry = Shuffleboard.getTab(tab).add(name, supplier())
      if (setter != null) {
        // Listen for changes to the entry if it is configurable
        shuffleboardEntry.entry.addListener(
          {
            val newValue = it.getEntry().value

            try {
              // Unchecked cast since we don't know the type of this
              // source due to type erasure
              @Suppress("UNCHECKED_CAST") setter(newValue.value as T)
            } catch (e: ClassCastException) {
              addEvent(
                "Logger",
                "Could not change value for $tab/$name due to invalid type cast ${
                e.message
                }.",
                Severity.ERROR
              )
            }
          },
          EntryListenerFlags.kUpdate
        )
      }
    } catch (e: IllegalArgumentException) {
      addEvent(
        "Logger", "Could not add $tab/$name to Shuffleboard due to invalid type", Severity.WARN
      )
    }
    dataSources.add(LogSource(tab, name, supplier, shuffleboardEntry, followSupplier))
  }

  /**
   * Add a source of data for the logger.
   *
   * @param tab The name of the Shuffleboard tab to add this value to. Typically the subsystem name.
   * @param name The name of this value.
   * @param supplier A function which returns the value to be logged.
   * @param setter An optional function which will be called when the value in Shuffleboard is
   * changed.
   */
  fun <T : Any> addSource(tab: String, name: String, supplier: () -> T) {
    addSource(tab, name, supplier, null)
  }

  /** Write logs to the CSV file. */
  fun saveLogs() {
    try {
      val data = "${Instant.now()},${DriverStation.getMatchTime()},$values"
      Files.write(file, listOf(data), StandardOpenOption.APPEND)
    } catch (e: Exception) {
      e.printStackTrace()
    }
  }

  /** Update values for logged data on Shuffleboard. */
  fun updateShuffleboard() {
    dataSources.forEach {
      if (it.shuffleboardWidget != null && it.followSupplier) {
        it.shuffleboardWidget.entry.setValue(it.supplier())
      }
    }
  }

  /** Begin logging. Creates CSV files and starts the logging thread. */
  fun startLogging() {
    createFile()
    logNotifier.startPeriodic(1.0)
  }

  @Throws(IOException::class)
  private fun saveTitles() {
    val titles = "Timestamp,match_time,${dataSources.joinToString(",") { "${it.tab}/${it.name}" }}"
    Files.write(file, listOf(titles), StandardOpenOption.APPEND)
  }

  /**
   * Track an event that occurs. Saves the event to a CSV and prints it to the console.
   *
   * @param source The source of the event, typically a subsystem or class name.
   * @param event The text to log.
   * @param severity The severity of the event. Defaults to INFO. Events with severity ERROR will be
   * logged to stderr instead of stdout.
   */
  fun addEvent(source: String, event: String, severity: Severity = Severity.INFO) {
    val log = "$severity,${Instant.now()},${DriverStation.getMatchTime()}," + "($source),$event"
    events.add(log)
    val consoleString =
      "[$severity][${Instant.now()}][${DriverStation.getMatchTime()}] " + "($source): $event"
    when (severity) {
      Severity.INFO -> println(consoleString)
      Severity.DEBUG -> println(consoleString)
      Severity.WARN -> println(consoleString)
      Severity.ERROR -> System.err.println(consoleString)
    }
  }

  private fun saveEvents() {
    while (events.isNotEmpty()) {
      try {
        val event = events.removeAt(0)
        Files.write(eventsFile, listOf(event), StandardOpenOption.APPEND)
      } catch (e: Exception) {
        e.printStackTrace()
      }
    }
  }
}
