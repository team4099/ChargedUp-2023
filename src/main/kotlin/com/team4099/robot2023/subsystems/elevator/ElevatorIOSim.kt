package com.team4099.robot2023.subsystems.elevator

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorIOSim : ElevatorIO {
  val elevatorSim: ElevatorSim =
    ElevatorSim(
      DCMotor.getNEO(2),
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.CARRIAGE_MASS.inKilograms,
      ElevatorConstants.SPOOL_RADIUS.inMeters,
      ElevatorConstants.ELEVATOR_MAX_RETRACTION.inMeters,
      ElevatorConstants.ELEVATOR_MAX_EXTENSION.inMeters,
      true
    )

  val elevatorController =
    PIDController<Meter, Volt>(
      ElevatorConstants.SIM_KP, ElevatorConstants.SIM_KI, ElevatorConstants.SIM_KD
    )

  init {

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    val m_mech2d = Mechanism2d(90.0, 90.0)
    val midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0.0)
    val MidNode =
      midNodeHome.append(
        MechanismLigament2d("Mid Cone Node", 34.0, 90.0, 10.0, Color8Bit(Color.kWhite))
      )
    val highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0.0)
    val HighNode =
      highNodeHome.append(
        MechanismLigament2d("High Cone Node", 46.0, 90.0, 10.0, Color8Bit(Color.kWhite))
      )
    val gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0.0)
    val GridNode =
      gridHome.append(
        MechanismLigament2d("Grid Wall", 49.75, 180.0, 50.0, Color8Bit(Color.kWhite))
      )
    val dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37.0)
    val DSRamp =
      dsHome.append(
        MechanismLigament2d(
          "Double Substation Ramp", 13.75, 180.0, 10.0, Color8Bit(Color.kWhite)
        )
      )

    val m_bumper =
      gridHome.append(MechanismLigament2d("Bumper", 30.5, 0.0, 60.0, Color8Bit(Color.kRed)))

    SmartDashboard.putData("Arm Sim", m_mech2d)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    elevatorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.elevatorPosition = elevatorSim.positionMeters.meters
    inputs.elevatorVelocity = elevatorSim.velocityMetersPerSecond.meters.perSecond

    inputs.leaderTempCelcius = 0.0.celsius
    inputs.leaderStatorCurrent = elevatorSim.currentDrawAmps.amps / 2
    inputs.leaderSupplyCurrent = 0.0.amps
    inputs.leaderAppliedOutput = 0.0

    inputs.followerTempCelcius = 0.0.celsius
    inputs.followerStatorCurrent = elevatorSim.currentDrawAmps.amps / 2
    inputs.followerSupplyCurrent = 0.0.amps
    inputs.followerAppliedOutput = 0.0

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.currentDrawAmps)
    )
  }

  override fun setOpenLoop(percentOutput: Double) {
    elevatorSim.setInputVoltage(RoboRioSim.getVInVoltage() * percentOutput)
  }

  override fun setPosition(height: Length, feedForward: ElectricalPotential) {
    val ff = MathUtil.clamp(feedForward.inVolts, -12.0, 12.0).volts
    val feedback = elevatorController.calculate(elevatorSim.positionMeters.meters, height)
    elevatorSim.setInputVoltage((ff + feedback).inVolts)
  }

  override fun zeroEncoder() {
    println("don't work right now")
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    elevatorController.setPID(kP, kI, kD)
  }
}
