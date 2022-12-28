package com.team4099.controller

import com.team4099.lib.controller.PIDController
import com.team4099.lib.units.Fraction
import com.team4099.lib.units.LinearVelocity
import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.Meter
import com.team4099.lib.units.base.Second
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.base.seconds
import com.team4099.lib.units.derived.ProportionalGain
import com.team4099.lib.units.perSecond
import kotlin.test.Test

class PIDControllerTest {

  @Test
  fun testConstruction() {
    val kP: ProportionalGain<Meter, Fraction<Meter, Second>> = 10.meters.perSecond / 1.meters
    val kD = 0.5.meters.perSecond / (1.meters / 1.seconds)
    val kI = 0.1.meters.perSecond / (10.meters * 1.seconds)
    val positionToVelocityPIDController = PIDController<Meter, Fraction<Meter, Second>>(kP, kI, kD)
  }
}
