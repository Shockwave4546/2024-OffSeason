package org.shockwave.subsystem.swerve

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var connected = false
    var yawPosition = Rotation2d()
    var yawVelocityRadPerSec = 0.0
    var odometryYawTimestamps = doubleArrayOf()
    var odometryYawPositions = arrayOf<Rotation2d>()

    override fun toLog(table: LogTable) {
      table.put("(1) Connected", connected)
      table.put("(2) Yaw Position (rad)", yawPosition.radians)
      table.put("(3) Yaw Velocity (rad/s)", yawVelocityRadPerSec)
      table.put("(4) Odometry Yaw Timestamps", odometryYawTimestamps)
      table.put("(5) Odometry Yaw Positions", *odometryYawPositions)
    }

    override fun fromLog(table: LogTable) {
      connected = table.get("(1) Connected", connected)
      yawPosition = Rotation2d(table.get("(2) Yaw Position (rad)", yawPosition.radians))
      yawVelocityRadPerSec = table.get("(3) Yaw Velocity (rad/s)", yawVelocityRadPerSec)
      odometryYawTimestamps = table.get("(4) Odometry Yaw Timestamps", odometryYawTimestamps)
      odometryYawPositions = table.get("(5) Odometry Yaw Positions", *odometryYawPositions)
    }
  }

  fun updateInputs(inputs: GyroIOInputs) {}
}