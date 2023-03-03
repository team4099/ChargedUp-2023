package com.team4099.lib.pathfollow

import org.team4099.lib.units.Acceleration
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Value
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import java.lang.Math.PI
import java.lang.Math.pow
import java.util.*
import kotlin.math.pow

class RotationSequence(
  private val sequence: TreeMap<Time, Angle>
) {

  fun sample(time: Time): State{
    var position = 0.0.radians
    var velocity = 0.0.radians.perSecond

    val lastPoint: Map.Entry<Time, Angle> = sequence.floorEntry(time)
    val nextPoint: Map.Entry<Time, Angle> = sequence.higherEntry(time)

    if (lastPoint == null && nextPoint == null){
      position = 0.0.radians
      velocity = 0.0.radians.perSecond
    } else if (lastPoint == null){
      position = nextPoint.value
      velocity = 0.0.radians.perSecond
    } else if (nextPoint == null){
      position = lastPoint.value
      velocity = 0.0.radians.perSecond
    } else {
      val acceleration: AngularAcceleration = (nextPoint.value - lastPoint.value) * 4 / (nextPoint.key - lastPoint.key) / 1.0.seconds
      if (time < ((nextPoint.key + lastPoint.key) / 2)){
        position = lastPoint.value + ((time - lastPoint.key).squared * (acceleration / 2)) as Angle
        velocity = (time - lastPoint.key) * acceleration
      } else {
        position = nextPoint.value + ((time - nextPoint.key).squared * (acceleration / 2)) as Angle
        velocity = (time - nextPoint.key) * acceleration
      }
    }

    while (position > Math.PI.radians){
      position -= (Math.PI * 2).radians
    }

    while (position < -Math.PI.radians){
      position += (Math.PI * 2).radians
    }

    return State(position, velocity)
  }



  companion object{
    data class State(val angle: Angle, val angularVelocity: AngularVelocity){

      constructor(): this(0.0.radians, 0.0.radians.perSecond) {}
    }
  }

}
