package com.team4099.lib

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Determines if another value is near this Double.
 *
 * @param around The value to compare this to.
 * @param tolerance The range within values will be considered near.
 * @return If [around] is within [tolerance] of this Double.
 */
fun Double.around(around: Double, tolerance: Double): Boolean {
  return abs(this - around) < tolerance
}

/**
 * Smoothly deadbands a value.
 *
 * @param deadband The range in which the value will be forced to zero.
 * @return This value adjusted to smoothly increase from zero if outside the deadband, zero if
 * inside the deadband.
 */
fun Double.smoothDeadband(deadband: Double): Double {
  return if (abs(this) < deadband) {
    0.0
  } else {
    (this - deadband) / (1 - deadband)
  }
}

/**
 * Constrains this Double between the given bounds.
 *
 * @param lowerBound The lower bound of this Double's range.
 * @param upperBound The upper bound of this Double's range.
 * @return Return this Double if it is in the range otherwise return [lowerBound] or [upperBound].
 */
fun Double.limit(lowerBound: Double, upperBound: Double): Double {
  return min(upperBound, max(lowerBound, this))
}

/**
 * Constrains this Int between the given bounds.
 *
 * @param lowerBound The lower bound of this Int's range.
 * @param upperBound The upper bound of this Int's range.
 * @return Return this Int if it is in the range otherwise return [lowerBound] or [upperBound].
 */
fun Int.limit(lowerBound: Int, upperBound: Int): Int {
  return min(upperBound, max(lowerBound, this))
}

/**
 * Linearly interpolate between two values.
 *
 * @param a The first value to interpolate between.
 * @param b The second value to interpolate between.
 * @param x The scalar that determines where the returned value falls between [a] and [b]. Limited
 * to between 0 and 1 inclusive.
 * @return A value between [a] and [b] determined by [x].
 */
fun interpolate(a: Double, b: Double, x: Double): Double {
  return a + (b - a) * x
}

/**
 * Linearly interpolate between two values.
 *
 * @param a The first value to interpolate between.
 * @param b The second value to interpolate between.
 * @param x The scalar that determines where the returned value falls between [a] and [b]. Limited
 * to between 0 and 1 inclusive.
 * @return A value between [a] and [b] determined by [x].
 */
fun <T : UnitKey> interpolate(a: Value<T>, b: Value<T>, x: Double): Value<T> {
  return Value(a.value + (b.value - a.value) * x)
}
