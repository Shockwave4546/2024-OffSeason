package org.shockwave.subsystem.swerve.gyro

import com.studica.frc.AHRS

class GyroIONavX : GyroIO {
  private val gyro = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz)

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.angle = gyro.angle
  }

  override fun reset() = gyro.reset()
  
  override fun getAngle() = gyro.angle
}