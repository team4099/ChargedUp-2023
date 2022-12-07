package com.team4099.lib.geometry

import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.meters
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

data class Quaternion(val m_r: Length, val m_v: Vector<N3>) {
  constructor() : this (
    1.0.meters,
    VecBuilder.fill(0.0, 0.0, 0.0)
    )

  constructor(w: Length, x: Double, y: Double, z: Double) : this (
    w,
    VecBuilder.fill(x, y, z)
    )

  operator fun times(other: Quaternion): Quaternion? {
    // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    val r1 = getW().inMeters
    val v1 = m_v
    val r2 = other.m_r.inMeters
    val v2 = other.m_v

    // v₁ x v₂
    val cross = VecBuilder.fill(
      v1[1, 0] * v2[2, 0] - v2[1, 0] * v1[2, 0],
      v2[0, 0] * v1[2, 0] - v1[0, 0] * v2[2, 0],
      v1[0, 0] * v2[1, 0] - v2[0, 0] * v1[1, 0])

    // v = r₁v₂ + r₂v₁ + v₁ x v₂
    val v: Matrix<N3, N1> = v2.times(r1).plus(v1.times(r2)).plus(cross)
    return Quaternion((r1 * r2 - v1.dot(v2)).meters, v.get(0, 0), v.get(1, 0), v.get(2, 0))
  }

  fun inverse(): Quaternion {
    return Quaternion(m_r, -m_v[0, 0], -m_v[1, 0], -m_v[2, 0])
  }

  fun normalize(): Quaternion {
    val norm = Math.sqrt(
      this.getW().inMeters * this.getW().inMeters +
      this.getX().inMeters * this.getX().inMeters +
      this.getY().inMeters * this.getY().inMeters +
      this.getZ().inMeters * this.getZ().inMeters
    )
    return if (norm == 0.0) {
      Quaternion()
    } else {
      Quaternion(this.getW() / norm, this.getX().inMeters / norm, this.getY().inMeters / norm, this.getZ().inMeters / norm)
    }
  }

  fun getW() : Length {
    return m_r
  }

  fun getX() : Length {
    return m_v.get(0, 0).meters
  }

  fun getY() : Length {
    return m_v.get(1, 0).meters
  }

  fun getZ() : Length {
    return m_v.get(2, 0).meters
  }

  fun toRotationVector(): Vector<N3> {
    // See equation (31) in "Integrating Generic Sensor Fusion Algorithms with
    // Sound State Representation through Encapsulation of Manifolds"
    //
    // https://arxiv.org/pdf/1107.1119.pdf
    val norm = m_v.norm()
    return if (norm < 1e-9) {
      m_v.times(2.0 / getW().inMeters - 2.0 / 3.0 * norm * norm / (getW().inMeters * getW().inMeters * getW().inMeters))
    } else {
      if (getW().inMeters < 0.0) {
        m_v.times(2.0 * Math.atan2(-norm, -getW().inMeters) / norm)
      } else {
        m_v.times(2.0 * Math.atan2(norm, getW().inMeters) / norm)
      }
    }
  }
}
