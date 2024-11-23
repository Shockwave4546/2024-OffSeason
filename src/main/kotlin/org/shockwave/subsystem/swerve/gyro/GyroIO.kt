package org.shockwave.subsystem.swerve.gyro

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var angle = 0.0

    override fun toLog(table: LogTable) {
      table.put("Angle", angle)
    }

    override fun fromLog(table: LogTable) {
      angle = table.get("Angle", angle)
    }
  }

  fun updateInputs(inputs: GyroIOInputs)

  fun reset()

  fun getAngle(): Double
}