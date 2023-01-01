package com.team4099.lib.logging

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value

class LoggedTunableValue<U : UnitKey>(
  val dashboardKey: String,
  val defaultValue: Value<U>,
  inline val conversionFunctions: Pair<(Value<U>) -> Double, (Double) -> Value<U>> =
    Pair({ it.value }, { Value(it) })
) {

  val tunableNumber = LoggedTunableNumber(dashboardKey, conversionFunctions.first(defaultValue))

  fun get(): Value<U> {
    return conversionFunctions.second(tunableNumber.get())
  }

  fun hasChanged(): Boolean {
    return tunableNumber.hasChanged()
  }
}
