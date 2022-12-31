package com.team4099.lib.units

import kotlin.math.absoluteValue

@JvmInline
value class Value<T : UnitKey>(internal val value: Double) : Comparable<Value<T>> {
  val absoluteValue: Value<T>
    get() = Value(value.absoluteValue)

  operator fun plus(o: Value<T>): Value<T> = Value(value + o.value)
  operator fun minus(o: Value<T>): Value<T> = Value(value - o.value)

  operator fun times(k: Double): Value<T> = Value(value * k)
  operator fun times(k: Number): Value<T> = this * k.toDouble()
  operator fun <K : UnitKey> times(o: Value<Fraction<K, T>>): Value<K> = Value(value * o.value)

  operator fun <K : UnitKey> times(o: Value<K>): Value<Product<T, K>> = Value(value * o.value)

  operator fun unaryMinus(): Value<T> = Value(-value)

  operator fun div(k: Double): Value<T> = Value(value / k)
  operator fun div(k: Number): Value<T> = this / k.toDouble()
  operator fun div(o: Value<T>): Double = value / o.value
  operator fun <K : UnitKey> div(o: Value<K>): Value<Fraction<T, K>> = Value(value / o.value)

  override operator fun compareTo(other: Value<T>): Int = value.compareTo(other.value)
}

infix fun <T : UnitKey> ClosedRange<Value<T>>.step(step: Value<T>): Iterable<Value<T>> {
  require(start.value.isFinite())
  require(endInclusive.value.isFinite())
  require(step.value > 0.0) { "Step must be positive, was: $step." }
  val sequence =
    generateSequence(start) { previous ->
      if (previous.value == Double.POSITIVE_INFINITY) return@generateSequence null
      val next = previous + step
      if (next > endInclusive) null else next
    }
  return sequence.asIterable()
}
