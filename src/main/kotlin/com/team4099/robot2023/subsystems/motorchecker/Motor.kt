package com.team4099.robot2023.subsystems.motorchecker

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.revrobotics.CANSparkMax
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.Temperature
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes

interface MotorType

typealias REVNeo = MotorType

typealias Falcon = MotorType

interface Motor<M : MotorType> {
  val temperature: Temperature
  val statorCurrent: Current

  fun setCurrentLimit(limit: Current) {}
}

class Neo(
  private val canSparkMax: CANSparkMax,
) : Motor<REVNeo> {
  override val statorCurrent: Current
    get() = canSparkMax.outputCurrent.amps

  override val temperature: Temperature
    get() = canSparkMax.motorTemperature.celsius

  override fun setCurrentLimit(limit: Current) {
    canSparkMax.setSmartCurrentLimit(limit.inAmperes.toInt())
  }
}

class Falcon500(private val falcon500: TalonFX) : Motor<Falcon> {
  override val temperature: Temperature
    get() = TODO("Not yet implemented")

  override val statorCurrent: Current
    get() = TODO("Not yet implemented")
}
