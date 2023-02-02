package com.team4099.robot2023.subsystems.groundintake

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.GroundIntakeConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inInches
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
import org.team4099.lib.units.derived.asDrivingOverDriven
import org.team4099.lib.units.derived.asKilogramsPerMeterSquared
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object GroundIntakeIOSim : GroundIntakeIO {

  val rollerSim =
    FlywheelSim(
      DCMotor.getNEO(1),
      GroundIntakeConstants.ROLLER_GEAR_RATIO.asDrivingOverDriven,
      GroundIntakeConstants.ROLLER_MOMENT_INERTIA.asKilogramsPerMeterSquared
    )

  val armSim =
    SingleJointedArmSim(
      DCMotor.getNEO(1),
      GroundIntakeConstants.ARM_OUTPUT_GEAR_RATIO.asDrivingOverDriven,
      GroundIntakeConstants.ARM_MOMENT_INTERTIA.asKilogramsPerMeterSquared,
      GroundIntakeConstants.ARM_LENGTH.inMeters,
      -15.degrees.inRadians,
      90.degrees.inRadians,
      GroundIntakeConstants.ARM_MASS.inKilograms,
      true
    )

  private val armController =
    PIDController(
      GroundIntakeConstants.PID.SIM_KP,
      GroundIntakeConstants.PID.SIM_KI,
      GroundIntakeConstants.PID.SIM_KD
    )

  private val m_mech2d = Mechanism2d(60.0, 60.0)
  private val m_armPivot = m_mech2d.getRoot("ArmPivot", 30.0, 30.0)
  private val m_armTower = m_armPivot.append(MechanismLigament2d("ArmTower", 30.0, -90.0))
  private val m_arm =
    m_armPivot.append(
      MechanismLigament2d(
        "Arm",
        GroundIntakeConstants.ARM_LENGTH.inInches,
        armSim.angleRads.radians.inDegrees,
        6.0,
        Color8Bit(Color.kYellow)
      )
    )

  init {
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d)
    m_armTower.setColor(Color8Bit(Color.kBlue))
  }

  override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
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

    m_arm.angle = armSim.angleRads.radians.inDegrees
  }

  /**
   * Sets the roller motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the roller motor to
   */
  override fun setRollerPower(voltage: ElectricalPotential) {
    rollerSim.setInputVoltage(
      MathUtil.clamp(
        voltage.inVolts,
        -GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts,
        GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts
      )
    )
  }

  /**
   * Sets the arm to a desired angle, uses feedforward to account for external forces in the system
   * The armPIDController uses the previously set PID constants and ff to calculate how to get to
   * the desired position
   *
   * @param armPosition the desired angle to set the aerm to
   * @param feedforward the amount of volts to apply for feedforward
   */
  override fun setArmPosition(armPosition: Angle, feedforward: ElectricalPotential) {
    val ff = MathUtil.clamp(feedforward.inVolts, -12.0, 12.0).volts
    val feedback = armController.calculate(armSim.angleRads.radians, armPosition)
    armSim.setInputVoltage((ff + feedback).inVolts)
  }

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  override fun setArmVoltage(voltage: ElectricalPotential) {
    armSim.setInputVoltage(
      MathUtil.clamp(
        voltage.inVolts,
        -GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts,
        GroundIntakeConstants.VOLTAGE_COMPENSATION.inVolts
      )
    )
  }

  /**
   * Updates the PID constants using the implementation controller, uses arm sensor to convert from
   * PID constants to motor controller units
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armController.setPID(kP, kI, kD)
  }

  /** recalculates the current position of the neo encoder using value from the absolute encoder */
  override fun zeroEncoder() {}
}
