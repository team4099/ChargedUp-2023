package com.team4099.robot2023.util

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.NumericalIntegration
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Mass
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.sin

class ElevatorSim(
  val gearbox: DCMotor,
  private val gearing: Double,
  private val carriageMassKg: Mass,
  private val drumRadiusMeters: Length,
  private val minHeightMeters: Length,
  private val maxHeightMeters: Length,
  private val angleElevation: Angle = 90.0.degrees,
  private val simulateGravity: Boolean,
) :
  ElevatorSim(
    gearbox,
    gearing,
    carriageMassKg.inKilograms,
    drumRadiusMeters.inMeters,
    minHeightMeters.inMeters,
    maxHeightMeters.inMeters,
    simulateGravity
  ) {

  override fun updateX(
    currentXhat: Matrix<N2, N1>?,
    u: Matrix<N1, N1>?,
    dtSeconds: Double
  ): Matrix<N2, N1> {
    // Calculate updated x-hat from Runge-Kutta.

    // Calculate updated x-hat from Runge-Kutta.
    val updatedXhat =
      NumericalIntegration.rkdp(
        { x: Matrix<N2, N1>, _u: Matrix<N1, N1> ->
          var xdot = m_plant.a.times(x).plus(m_plant.b.times(_u))
          if (simulateGravity) {
            xdot = xdot.plus(VecBuilder.fill(0.0, -9.8 * angleElevation.sin))
          }
          xdot
        },
        currentXhat,
        u,
        dtSeconds
      )

    // We check for collisions after updating x-hat.

    // We check for collisions after updating x-hat.
    if (wouldHitLowerLimit(updatedXhat[0, 0])) {
      return VecBuilder.fill(minHeightMeters.inMeters, 0.0)
    }
    return if (wouldHitUpperLimit(updatedXhat[0, 0])) {
      VecBuilder.fill(maxHeightMeters.inMeters, 0.0)
    } else updatedXhat
  }
}
