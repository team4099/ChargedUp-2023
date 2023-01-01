package com.team4099.lib.logging

import com.team4099.lib.units.UnitKey
import com.team4099.lib.units.Value

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard
 *
 * @param dashboardKey Key associated with value
 * @param defaultValue Default value of the Tunable Number
 * @param conversionFunctions.first Defines how to go from the Tunable Value (in Units Library) to a
 * double
 * @param conversionFunctions.second Defines how to go from the double in dashboard to the Tunable
 * Value
 */
class LoggedTunableValue<U : UnitKey>(
  val dashboardKey: String,
  var defaultValue: Value<U>,
  inline val conversionFunctions: Pair<(Value<U>) -> Double, (Double) -> Value<U>> =
    Pair({ it.value }, { Value(it) })
) {

  var tunableNumber = LoggedTunableNumber(dashboardKey, conversionFunctions.first(defaultValue))

  fun get(): Value<U> {
    return conversionFunctions.second(tunableNumber.get())
  }

  fun hasChanged(): Boolean {
    return tunableNumber.hasChanged()
  }

  fun initDefault(defaultValue: Value<U>) {
    tunableNumber.initDefault(conversionFunctions.first(defaultValue))
  }
}
