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

data class Quaternion(val m_r: Angle, val m_v: Vector<N3>) {
  constructor() : this(1.0.radians, VecBuilder.fill(0.0, 0.0, 0.0))

  constructor(w: Angle, x: Double, y: Double, z: Double) : this(w, VecBuilder.fill(x, y, z))

  constructor(m_q: QuaternionWPILIB) : this(m_q.w.radians, VecBuilder.fill(m_q.x, m_q.y, m_q.z))

  operator fun times(other: Quaternion): Quaternion {
    return Quaternion(quaternion * other.quaternion)
  }

  fun inverse(): Quaternion {
    return Quaternion(m_r, -m_v[0, 0], -m_v[1, 0], -m_v[2, 0])
  }

  fun normalize(): Quaternion {
    return Quaternion(quaternion.normalize())
  }

  val w: Angle = m_r

  val x: Length = m_v[0, 0].meters

  val y: Length = m_v[1, 0].meters

  val z: Length = m_v[2, 0].meters

  val quaternion: QuaternionWPILIB =
    QuaternionWPILIB(w.inRadians, x.inMeters, y.inMeters, z.inMeters)

  val rotationVector: Vector<N3> = quaternion.toRotationVector()
}
