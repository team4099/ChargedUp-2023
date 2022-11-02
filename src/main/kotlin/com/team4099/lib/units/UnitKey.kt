package com.team4099.lib.units

interface UnitKey

object Unitless : UnitKey

class Product<A : UnitKey, B : UnitKey> : UnitKey

class Fraction<N : UnitKey, D : UnitKey> : UnitKey

operator fun <T : UnitKey, K : UnitKey> Value<Fraction<K, T>>.times(o: Value<T>): Value<K> =
  Value(value * o.value)

typealias Inverse<K> = Fraction<Unitless, K>

typealias Squared<K> = Product<K, K>

typealias Cubed<K> = Product<K, Squared<K>>

val Double.unitless: Value<Unitless>
  get() = Value(this)

val Number.unitless: Value<Unitless>
  get() = this.toDouble().unitless

val Value<Unitless>.asDouble
  get() = value
