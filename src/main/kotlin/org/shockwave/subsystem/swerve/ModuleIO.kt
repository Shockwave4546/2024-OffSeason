package org.shockwave.subsystem.swerve

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ModuleIO {
  class ModuleIOInputs : LoggableInputs {
    var driveConnected = false
    var drivePositionRad = 0.0
    var driveVelocityMetersPerSec = 0.0
    var driveAppliedVolts = 0.0
    var driveCurrentAmps = 0.0

    var turnConnected = false
    var turnPosition = Rotation2d()
    var turnVelocityRadPerSec = 0.0
    var turnAppliedVolts = 0.0
    var turnCurrentAmps = 0.0

    var odometryTimestamps = doubleArrayOf()
    var odometryDrivePositionsMeters = doubleArrayOf()
    var odometryTurnPositions = arrayOf<Rotation2d>()

    override fun toLog(table: LogTable) {
      table.put("(1) Drive Connected", driveConnected)
      table.put("(2) Drive Position", drivePosition)
      table.put("(3) Drive Velocity (mps)", driveVelocityMetersPerSec)
      table.put("(4) Drive Applied Volts", driveAppliedVolts)
      table.put("(5) Drive Current (A)", driveCurrentAmps)

      table.put("(6) Turn Connected", turnConnected)
      table.put("(7) Turn Position (rad)", turnPosition.radians)
      table.put("(8) Turn Velocity (rad per s)", turnVelocityRadPerSec)
      table.put("(9) Turn Applied Volts", turnAppliedVolts)
      table.put("(10) Turn Current (A)", turnCurrentAmps)

      table.put("(11) Odometry Timestamps", odometryTimestamps)
      table.put("(12) Odometry Drive Positions (m)", odometryDrivePositionsMeters)
      table.put("(13) Odometry Turn Positions", *odometryTurnPositions)
    }

    override fun fromLog(table: LogTable) {
      driveConnected = table.get("(1) Drive Connected", driveConnected)
      drivePosition = table.get("(2) Drive Position", drivePosition)
      driveVelocityMetersPerSec = table.get("(3) Drive Velocity (mps)", driveVelocityMetersPerSec)
      driveAppliedVolts = table.get("(4) Drive Applied Volts", driveAppliedVolts)
      driveCurrentAmps = table.get("(5) Drive Current (A)", driveCurrentAmps)

      turnConnected = table.get("(6) Turn Connected", turnConnected)
      turnPosition = Rotation2d(table.get("(7) Turn Position (rad)", turnPosition.radians))
      turnVelocityRadPerSec = table.get("(8) Turn Velocity (rad per s)", turnVelocityRadPerSec)
      turnAppliedVolts = table.get("(9) Turn Applied Volts", turnAppliedVolts)
      turnCurrentAmps = table.get("(10) Turn Current (A)", turnCurrentAmps)

      odometryTimestamps = table.get("(11) Odometry Timestamps", odometryTimestamps)
      odometryDrivePositionsMeters = table.get("(12) Odometry Drive Positions (rad)", odometryDrivePositionsMeters)
      odometryTurnPositions = table.get("(13) Odometry Turn Positions", *odometryTurnPositions)
    }
  }

  fun updateInputs(inputs: ModuleIOInputs) {}

  fun setDriveVoltage(voltage: Double) {}

  fun setTurnVoltage(voltage: Double) {}

  fun setDriveVelocity(metersPerSecond: Double) {}

  fun setTurnPosition(rotation: Rotation2d) {}
}