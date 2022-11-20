package com.team4099.robot2022.subsytems.elevator

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.lib.units.base.Length
import com.team4099.lib.units.base.amps
import com.team4099.lib.units.base.inInches
import com.team4099.lib.units.base.inKilograms
import com.team4099.lib.units.base.inMeters
import com.team4099.lib.units.base.inSeconds
import com.team4099.lib.units.base.meters
import com.team4099.lib.units.derived.ElectricalPotential
import com.team4099.lib.units.derived.inVolts
import com.team4099.lib.units.perSecond
import com.team4099.lib.units.sparkMaxLinearMechanismSensor
import com.team4099.robot2022.config.constants.Constants
import com.team4099.robot2022.config.constants.ElevatorConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/** Elevator subsystem implementation using WPILIB simulation */
object ElevatorIOSim : ElevatorIO {

  private val elevatorGearbox: DCMotor = DCMotor.getNEO(2)
  val simulatedElevator: ElevatorSim =
    ElevatorSim(
      elevatorGearbox,
      ElevatorConstants.GEARING,
      ElevatorConstants.CARRIAGE_MASS.inKilograms,
      ElevatorConstants.DRUM_RADIUS.inMeters,
      ElevatorConstants.elevatorMinExtension.inMeters,
      ElevatorConstants.MAX_HEIGHT_OF_TOP_OF_CARRIAGE.inMeters,
      true
    )

  // simulate a motor and control it and use its outputs to control elevator
  private val simulatedMotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val elevatorSensor =
    sparkMaxLinearMechanismSensor(
      simulatedMotor, ElevatorConstants.GEARING, ElevatorConstants.DRUM_RADIUS
    )

  private val elevatorMechanism2d: MechanismLigament2d
  init {
    val mech2d = Mechanism2d(20.0, 100.0)
    val mech2dRoot = mech2d.getRoot("Robot", 10.0, 0.0)
    elevatorMechanism2d =
      MechanismLigament2d("Elevator", simulatedElevator.positionMeters.meters.inInches, 90.0)
    mech2dRoot.append(elevatorMechanism2d)

    // publishing the mechanism visualization to SmartDashboard
    SmartDashboard.putData("Elevator Simulation Visualization", mech2d)
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
    simulatedElevator.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.position = simulatedElevator.positionMeters.meters
    inputs.velocity = simulatedElevator.velocityMetersPerSecond.meters.perSecond

    // WPILIB multiplies current draws by `numMotors`. To get the draw for each theoretical motor
    // you divide by `numMotors`
    inputs.leaderSupplyCurrentDraw = (simulatedElevator.currentDrawAmps / 2).amps
    inputs.followerSupplyCurrentDraw = (simulatedElevator.currentDrawAmps / 2).amps

    // Setting a more accurate simulated voltage under "load"
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(simulatedElevator.currentDrawAmps)
    )

    // Updating elevator visualization with simulated position
    elevatorMechanism2d.length = simulatedElevator.positionMeters.meters.inInches
  }

  override fun setOpenLoop(percentOutput: Double) {
    // TODO figure out if you need to use robot simulated voltage
    val appliedVolts = 12.0 * MathUtil.clamp(percentOutput, -1.0, 1.0)
    simulatedElevator.setInputVoltage(appliedVolts)
  }

  override fun setPosition(height: Length, feedforward: ElectricalPotential) {
    simulatedMotor.pidController.setReference(
      elevatorSensor.positionToRawUnits(height),
      CANSparkMax.ControlType.kSmartMotion,
      0,
      feedforward.inVolts
    )
  }
}
