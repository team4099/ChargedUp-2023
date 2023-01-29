package com.team4099.robot2023.subsystems.intake

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object IntakeIOSim : IntakeIO {

  val rollerSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      IntakeConstants.ROLLER_GEAR_RATIO,
      IntakeConstants.ROLLER_MOMENT_INERTIA
    )

  val armSim =
    SingleJointedArmSim(
      DCMotor.getNEO(1),
      IntakeConstants.ARM_OUTPUT_GEAR_RATIO,
      IntakeConstants.ARM_MOMENT_INTERTIA,
      IntakeConstants.ARM_LENGTH.inMeters,
      IntakeConstants.ARM_MIN_ROTATION.inRadians,
      IntakeConstants.ARM_MAX_ROTATION.inRadians,
      IntakeConstants.ARM_MASS.inKilograms,
      true
    )

  private val armController =
    PIDController(IntakeConstants.SIM_KP, IntakeConstants.SIM_KI, IntakeConstants.SIM_KD)

  init {}

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerSupplyCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerAppliedVoltage = 0.volts
    inputs.rollerStatorCurrent = 0.amps
    inputs.rollerTemp = 0.0.celsius

    inputs.armPosition = armSim.angleRads.radians
    inputs.armVelocity = armSim.velocityRadPerSec.radians.perSecond
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
  }

  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    val ff = MathUtil.clamp(feedforward.inVolts, -12.0, 12.0).volts
    val feedback = armController.calculate(armSim.angleRads.radians, armPosition)
    armSim.setInputVoltage((ff + feedback).inVolts)
  }

  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSim.setInputVoltage(MathUtil.clamp(voltage.inVolts, -12.0, 12.0))
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armController.setPID(kP, kI, kD)
  }

  override fun zeroEncoder() {}
}
