package org.shockwave.subsystem.swerve.module

import com.revrobotics.REVLibError
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.shockwave.utils.PIDFGains

interface ModuleIO {
  class ModuleIOInputs : LoggableInputs {
    var drivePosition = 0.0 // m
    var driveVelocity = 0.0 // m/s
    var driveAppliedVolts = 0.0
    var driveCurrent = 0.0
    var driveTemp = 0.0 // C

    var rotPosition = Rotation2d()
    var rotVelocity = 0.0 // rad/s
    var rotAppliedVolts = 0.0
    var rotCurrent = 0.0
    var rotTemp = 0.0 // C

    override fun toLog(table: LogTable) {
      table.put("Drive/(1) Position", drivePosition)
      table.put("Drive/(2) Velocity", driveVelocity)
      table.put("Drive/(3) AppliedVolts", driveAppliedVolts)
      table.put("Drive/(4) Current", driveCurrent)
      table.put("Drive/(5) Temp", driveTemp)

      table.put("Rot/(1) Position", rotPosition.radians)
      table.put("Rot/(2) Velocity", rotVelocity)
      table.put("Rot/(3) AppliedVolts", rotAppliedVolts)
      table.put("Rot/(4) Current", rotCurrent)
      table.put("Rot/(5) Temp", rotTemp)
    }

    override fun fromLog(table: LogTable) {
      drivePosition = table.get("Drive/(1) Position", drivePosition)
      driveVelocity = table.get("Drive/(2) Velocity", driveVelocity)
      driveAppliedVolts = table.get("Drive/(3) AppliedVolts", driveAppliedVolts)
      driveCurrent = table.get("Drive/(4) Current", driveCurrent)
      driveTemp = table.get("Drive/(5) Temp", driveTemp)

      rotPosition = Rotation2d(table.get("Rot/(1) Position", rotPosition.radians))
      rotVelocity = table.get("Rot/(2) Velocity", rotVelocity)
      rotAppliedVolts = table.get("Rot/(3) AppliedVolts", rotAppliedVolts)
      rotCurrent = table.get("Rot/(4) Current", rotCurrent)
      rotTemp = table.get("Rot/(5) Temp", rotTemp)
    }
  }

  fun updateInputs(inputs: ModuleIOInputs)

  fun getState(): SwerveModuleState

  fun getPosition(): SwerveModulePosition

  fun setDesiredState(desiredState: SwerveModuleState)

  fun resetDriveEncoder(): REVLibError

  fun setDrivePIDF(pidf: PIDFGains): REVLibError

  fun setRotPIDF(pidf: PIDFGains): REVLibError
}