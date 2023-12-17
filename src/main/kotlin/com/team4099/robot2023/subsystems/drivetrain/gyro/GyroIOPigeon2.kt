package com.team4099.robot2023.subsystems.drivetrain.gyro

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.GyroConstants
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.threads.PhoenixOdometryThread
import com.team4099.robot2023.subsystems.drivetrain.swervemodule.threads.SparkMaxOdometryThread
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import java.util.Queue
import kotlin.math.IEEErem

object GyroIOPigeon2 : GyroIO {
  private var pigeon2 = Pigeon2(Constants.Gyro.PIGEON_2_ID, Constants.Universal.CANIVORE_NAME)

  private val isConnected: Boolean = pigeon2.upTime.value > 0.0

  var gyroYawOffset: Angle = 0.0.degrees
  var gyroPitchOffset: Angle = 0.0.degrees
  var gyroRollOffset: Angle = 0.0.degrees

  val yawPositionQueue: Queue<Double>

  val rawGyro: Angle = 0.0.degrees

  /** The current angle of the drivetrain. */
  val gyroYaw: Angle
    get() {
      if (isConnected) {
        var rawYaw = pigeon2.yaw.value + gyroYawOffset.inDegrees
        rawYaw += DrivetrainConstants.GYRO_RATE_COEFFICIENT * gyroYawRate.inDegreesPerSecond
        return rawYaw.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroPitch: Angle
    get() {
      if (isConnected) {
        val rawPitch = pigeon2.pitch.value + gyroPitchOffset.inDegrees
        return rawPitch.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroRoll: Angle
    get() {
      if (isConnected) {
        val rawRoll = pigeon2.roll.value + gyroRollOffset.inDegrees
        return rawRoll.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
      }
    }

  val gyroYawRate: AngularVelocity
    get() {
      if (isConnected) {
        return pigeon2.angularVelocityZWorld.value.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroPitchRate: AngularVelocity
    get() {
      if (isConnected) {
        return pigeon2.angularVelocityYWorld.value.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroRollRate: AngularVelocity
    get() {
      if (isConnected) {
        return pigeon2.angularVelocityXWorld.value.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  init {
    val pigeon2Configuration = Pigeon2Configuration()
    pigeon2Configuration.MountPose.MountPosePitch = GyroConstants.mountPitch.inRadians
    pigeon2Configuration.MountPose.MountPoseYaw = GyroConstants.mountYaw.inRadians
    pigeon2Configuration.MountPose.MountPoseRoll = GyroConstants.mountRoll.inRadians

    yawPositionQueue =
      if (Constants.Drivetrain.DRIVETRAIN_TYPE ==
        Constants.Drivetrain.DrivetrainType.PHOENIX_TALON
      ) {
        PhoenixOdometryThread.getInstance().registerSignal(pigeon2, pigeon2.getYaw())
      } else {
        SparkMaxOdometryThread.getInstance().registerSignal { pigeon2.yaw.getValueAsDouble() }
      }

    // TODO look into more pigeon configuration stuff
    pigeon2.configurator.apply(pigeon2Configuration)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {

    inputs.rawGyroYaw = pigeon2.yaw.value.degrees

    inputs.gyroConnected = isConnected

    inputs.gyroYaw = gyroYaw
    inputs.gyroPitch = gyroPitch
    inputs.gyroRoll = gyroRoll

    inputs.gyroYawRate = gyroYawRate
    inputs.gyroPitchRate = gyroPitchRate
    inputs.gyroRollRate = gyroRollRate

    inputs.odometryYawPositions =
      yawPositionQueue.stream().map { value: Double -> value.degrees }.toArray() as Array<Angle>
    yawPositionQueue.clear()

    Logger.recordOutput("Gyro/rawYawDegrees", pigeon2.yaw.value)
  }

  override fun zeroGyroYaw(toAngle: Angle) {
    gyroYawOffset = toAngle - pigeon2.yaw.value.IEEErem(360.0).degrees
  }

  override fun zeroGyroPitch(toAngle: Angle) {
    gyroPitchOffset = toAngle - pigeon2.pitch.value.IEEErem(360.0).degrees
  }

  override fun zeroGyroRoll(toAngle: Angle) {
    gyroRollOffset = toAngle - pigeon2.roll.value.IEEErem(360.0).degrees
  }
}
