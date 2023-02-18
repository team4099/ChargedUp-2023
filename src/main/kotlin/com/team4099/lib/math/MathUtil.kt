package com.team4099.lib.math

import org.team4099.lib.units.UnitKey
import org.team4099.lib.units.Value

fun <T : UnitKey> clamp(input: Value<T>, lowerBound: Value<T>, upperBound: Value<T>): Value<T> {
  return maxOf(lowerBound, minOf(input, upperBound))
}
