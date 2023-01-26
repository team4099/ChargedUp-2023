package com.team4099.robot2023.subsystems.manipulator

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ManipulatorConstants
import com.team4099.robot2023.util.ElevatorSim
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorIOSim : ManipulatorIO {

  val rollerSim: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      ManipulatorConstants.ROLLER_GEAR_RATIO,
      ManipulatorConstants.MOMENT_INERTIA
    )

  val armSim: ElevatorSim =
    ElevatorSim(
      DCMotor.getNEO(1),
      ManipulatorConstants.ARM_GEAR_RATIO,
      ManipulatorConstants.ARM_MASS,
      ManipulatorConstants.ARM_SPOOL_RADIUS,
      ManipulatorConstants.ARM_MAX_RETRACTION,
      ManipulatorConstants.ARM_MAX_EXTENSION,
      simulateGravity = false
    )

  private val armController =
    PIDController(
      ManipulatorConstants.SIM_ARM_KP,
      ManipulatorConstants.SIM_ARM_KI,
      ManipulatorConstants.SIM_ARM_KD
    )

  val mechanism_widget: Mechanism2d = Mechanism2d(700.0, 700.0, Color8Bit(Color.kBlack))
  val mechanism_root: MechanismRoot2d = mechanism_widget.getRoot("pivot", 350.0, 0.0)
  val mechanism_arm: MechanismLigament2d =
    mechanism_root.append(MechanismLigament2d("arm", 700.0, 0.0, 8.0, Color8Bit(Color.kWhite)))

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.radians.perSecond
    inputs.rollerSupplyCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerPosition = 0.degrees
    inputs.rollerAppliedVoltage = 0.volts
    inputs.rollerStatorCurrent = 0.amps
    inputs.rollerTemp = 0.0.celsius

    inputs.armPosition = armSim.positionMeters.meters
    inputs.armVelocity = armSim.velocityMetersPerSecond.meters.perSecond
    inputs.armAppliedVoltage = 0.volts
    inputs.armStatorCurrent = armSim.currentDrawAmps.amps
    inputs.armSupplyCurrent = 0.amps
    inputs.armTemp = 0.celsius
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.currentDrawAmps)
    )
  }

  override fun setRollerPower(voltage: ElectricalPotential) {
    rollerSim.setInputVoltage(MathUtil.clamp(voltage.inVolts, -12.0, 12.0))
    setMechanismNodePosition(voltage / Constants.Universal.VOLTAGE_COMPENSATION)
  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSim.setInputVoltage(MathUtil.clamp(voltage.inVolts, -12.0, 12.0))
  }

  override fun setPosition(position: Length, feedforward: ElectricalPotential) {
    val ff = MathUtil.clamp(feedforward.inVolts, -12.0, 12.0).volts
    val feedback = armController.calculate(armSim.positionMeters.meters, position)
    armSim.setInputVoltage((ff + feedback).inVolts)
  }

  override fun zeroEncoder() {}

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    armController.setPID(kP, kI, kD)
  }

  fun setMechanismNodePosition(scale: Double) {
    mechanism_root.setPosition(350.0 + 350.0 * scale, 0.0)
  }
}
