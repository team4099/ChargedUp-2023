package com.team4099.lib.math

import org.team4099.lib.units.UnitKey
import org.team4099.lib.units.Value
import java.util.Arrays
import kotlin.collections.ArrayList

class CircularBuffer<U : UnitKey>(size: Int) {
  private var m_data: ArrayList<Value<U>>

  // Index of element at front of buffer
  private var m_front = 0

  // Number of elements used in buffer
  private var m_length = 0

  /**
   * Create a CircularBuffer with the provided size.
   *
   * @param size The size of the circular buffer.
   */
  init {
    m_data = ArrayList(size)
  }

  /**
   * Returns number of elements in buffer.
   *
   * @return number of elements in buffer
   */
  fun size(): Int {
    return m_length
  }

  val first: Value<U>
    /**
     * Get value at front of buffer.
     *
     * @return value at front of buffer
     */
    get() = m_data[m_front]

  val last: Value<U>
    /**
     * Get value at back of buffer.
     *
     * @return value at back of buffer
     */
    get() =// If there are no elements in the buffer, do nothing
      if (m_length == 0) {
        Value(0.0)
      } else m_data[(m_front + m_length - 1) % m_data.size]

  /**
   * Push new value onto front of the buffer. The value at the back is overwritten if the buffer is
   * full.
   *
   * @param value The value to push.
   */
  fun addFirst(value: Value<U>) {
    if (m_data.isEmpty()) {
      return
    }
    m_front = moduloDec(m_front)
    m_data[m_front] = value
    if (m_length < m_data.size) {
      m_length++
    }
  }

  /**
   * Push new value onto back of the buffer. The value at the front is overwritten if the buffer is
   * full.
   *
   * @param value The value to push.
   */
  fun addLast(value: Value<U>) {
    if (m_data.size == 0) {
      return
    }
    m_data[(m_front + m_length) % m_data.size] = value
    if (m_length < m_data.size) {
      m_length++
    } else {
      // Increment front if buffer is full to maintain size
      m_front = moduloInc(m_front)
    }
  }

  /**
   * Pop value at front of buffer.
   *
   * @return value at front of buffer
   */
  fun removeFirst(): Value<U> {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
      return Value(0.0)
    }
    val temp = m_data[m_front]
    m_front = moduloInc(m_front)
    m_length--
    return temp
  }

  /**
   * Pop value at back of buffer.
   *
   * @return value at back of buffer
   */
  fun removeLast(): Value<U> {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
      return Value(0.0)
    }
    m_length--
    return m_data[(m_front + m_length) % m_data.size]
  }

  /**
   * Resizes internal buffer to given size.
   *
   *
   * A new buffer is allocated because arrays are not resizable.
   *
   * @param size New buffer size.
   */
  fun resize(size: Int) {
    val newBuffer = ArrayList<Value<U>>(size)
    m_length = Math.min(m_length, size)
    for (i in 0 until m_length) {
      newBuffer[i] = m_data[(m_front + i) % m_data.size]
    }
    m_data = newBuffer
    m_front = 0
  }

  /** Sets internal buffer contents to zero.  */
  fun clear() {
    m_data.clear()
    m_front = 0
    m_length = 0
  }

  /**
   * Get the element at the provided index relative to the start of the buffer.
   *
   * @param index Index into the buffer.
   * @return Element at index starting from front of buffer.
   */
  operator fun get(index: Int): Value<U> {
    return m_data[(m_front + index) % m_data.size]
  }

  /**
   * Increment an index modulo the length of the m_data buffer.
   *
   * @param index Index into the buffer.
   */
  private fun moduloInc(index: Int): Int {
    return (index + 1) % m_data.size
  }

  /**
   * Decrement an index modulo the length of the m_data buffer.
   *
   * @param index Index into the buffer.
   */
  private fun moduloDec(index: Int): Int {
    return if (index == 0) {
      m_data.size - 1
    } else {
      index - 1
    }
  }
}
