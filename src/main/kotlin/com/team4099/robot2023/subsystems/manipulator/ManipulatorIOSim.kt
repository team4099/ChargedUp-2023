package com.team4099.robot2023.subsystems.manipulator

import com.team4099.robot2023.config.constants.ManipulatorConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorIOSim : ManipulatorIO {

  val manipulatorMotor: FlywheelSim =
    FlywheelSim(
      DCMotor.getNEO(1), ManipulatorConstants.GEAR_RATIO, ManipulatorConstants.MOMENT_INERTIA
    )
  val mechanism_widget: Mechanism2d = Mechanism2d(700.0, 700.0, Color8Bit(Color.kBlack))
  val mechanism_root: MechanismRoot2d = mechanism_widget.getRoot("pivot", 350.0, 0.0)
  val mechanism_arm: MechanismLigament2d =
    mechanism_root.append(MechanismLigament2d("arm", 700.0, 0.0, 8.0, Color8Bit(Color.kWhite)))

  override fun setRollerPower(percentOutput: Double) {
    manipulatorMotor.setInputVoltage(MathUtil.clamp(percentOutput, -12.0, 12.0))
    setMechanismNodePosition(percentOutput)
  }

  override fun updateInputs(inputs: ManipulatorIO.ManipulatorIOInputs) {
    inputs.rollerVelocity = manipulatorMotor.angularVelocityRPM.radians.perSecond
    inputs.rollerSupplyCurrent = manipulatorMotor.currentDrawAmps.amps

    inputs.rollerPosition = 0.degrees
    inputs.rollerAppliedVoltage = 0.volts
    inputs.rollerStatorCurrent = 0.amps
    inputs.rollerTempCelcius = 0.0.celsius
  }

  fun setMechanismNodePosition(scale: Double) {
    mechanism_root.setPosition(350.0 + 350.0 * scale, 0.0)
  }
}
