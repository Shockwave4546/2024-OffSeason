package org.shockwave.subsystem.swervenew

import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units

class GyroIONavX : GyroIO {
  private val navX = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k100Hz)
  private val yawPositionQueue = SparkOdometryThread.registerSignal(navX::getAngle)
  private val yawTimestampQueue = SparkOdometryThread.makeTimestampQueue()

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = navX.isConnected
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.angle)
    inputs.yawVelocityRadPerSec = Units.degreesToRadians((-navX.rawGyroZ).toDouble())

    inputs.odometryYawTimestamps =
      yawTimestampQueue.stream().mapToDouble { it }.toArray()
    inputs.odometryYawPositions =
      yawPositionQueue.stream()
        .map { Rotation2d.fromDegrees(-it) }
        .toArray { arrayOf() }
    yawTimestampQueue.clear()
    yawPositionQueue.clear()
  }
}