package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.Angle
import com.team4099.lib.units.derived.inRadians
import com.team4099.lib.units.derived.radians
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N3

data class Quaternion(val w: Angle, val v: Vector<N3>) {
  val x: Length = v[0, 0].meters

  val y: Length = v[1, 0].meters

  val z: Length = v[2, 0].meters

  val quaternion: QuaternionWPILIB =
    QuaternionWPILIB(w.inRadians, x.inMeters, y.inMeters, z.inMeters)

  val rotationVector: Vector<N3> = quaternion.toRotationVector()

  constructor() : this(1.0.radians, VecBuilder.fill(0.0, 0.0, 0.0))

  constructor(w: Angle, x: Double, y: Double, z: Double) : this(w, VecBuilder.fill(x, y, z))

  constructor(
    quaternion: QuaternionWPILIB
  ) : this(quaternion.w.radians, VecBuilder.fill(quaternion.x, quaternion.y, quaternion.z))

  operator fun times(other: Quaternion): Quaternion {
    return Quaternion(quaternion * other.quaternion)
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Quaternion) return false

    if ((w * other.w.inRadians + v.dot(other.v).radians).absoluteValue.value < 1.0 - 1E-9)
      return false

    return true
  }

  fun inverse(): Quaternion {
    return Quaternion(w, -v[0, 0], -v[1, 0], -v[2, 0])
  }

  fun normalize(): Quaternion {
    return Quaternion(quaternion.normalize())
  }
}
