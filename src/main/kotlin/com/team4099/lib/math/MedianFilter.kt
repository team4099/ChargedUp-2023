package com.team4099.lib.math

import org.team4099.lib.units.UnitKey
import org.team4099.lib.units.Value
import java.util.*
import kotlin.collections.ArrayList

/**
 * A class that implements a moving-window median filter. Useful for reducing measurement noise,
 * especially with processes that generate occasional, extreme outliers (such as values from vision
 * processing, LIDAR, or ultrasonic sensors).
 */
class MedianFilter<U: UnitKey>(private val m_size: Int) {
  private val m_valueBuffer: CircularBuffer<U> = CircularBuffer(m_size)
  private val m_orderedValues: MutableList<Value<U>>

  /**
   * Creates a new MedianFilter.
   *
   * @param size The number of samples in the moving window.
   */
  init {
    // Circular buffer of values currently in the window, ordered by time
    // List of values currently in the window, ordered by value
    m_orderedValues = ArrayList(m_size)
    // Size of rolling window
  }

  /**
   * Calculates the moving-window median for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The median of the moving window, updated to include the next value.
   */
  fun calculate(next: Value<U>): Value<U> {
    // Find insertion point for next value
    var index = Collections.binarySearch(m_orderedValues, next)

    // Deal with binarySearch behavior for element not found
    if (index < 0) {
      index = Math.abs(index + 1)
    }

    // Place value at proper insertion point
    m_orderedValues.add(index, next)
    var curSize = m_orderedValues.size

    // If buffer is at max size, pop element off of end of circular buffer
    // and remove from ordered list
    if (curSize > m_size) {
      m_orderedValues.remove(m_valueBuffer.removeLast())
      --curSize
    }

    // Add next value to circular buffer
    m_valueBuffer.addFirst(next)
    return if (curSize % 2 != 0) {
      // If size is odd, return middle element of sorted list
      m_orderedValues[curSize / 2]
    } else {
      // If size is even, return average of middle elements
      (m_orderedValues[curSize / 2 - 1] + m_orderedValues[curSize / 2]) / 2.0
    }
  }

  /** Resets the filter, clearing the window of all elements.  */
  fun reset() {
    m_orderedValues.clear()
    m_valueBuffer.clear()
  }
}
