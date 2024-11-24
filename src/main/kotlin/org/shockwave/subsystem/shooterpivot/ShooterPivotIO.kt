package org.shockwave.subsystem.shooterpivot

import com.revrobotics.REVLibError
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.shockwave.utils.PIDFGains

interface ShooterPivotIO {
  class ShooterPivotIOInputs : LoggableInputs {
    var angle = 0.0
    var appliedVolts = 0.0
    var current = 0.0
    var temp = 0.0

    override fun toLog(table: LogTable) {
      table.put("1.Angle", angle)
      table.put("2.AppliedVolts", appliedVolts)
      table.put("3.Current", current)
      table.put("4.Temp", temp)
    }

    override fun fromLog(table: LogTable) {
      angle = table.get("1.Angle", angle)
      appliedVolts = table.get("2.AppliedVolts", appliedVolts)
      current = table.get("3.Current", current)
      temp = table.get("4.Temp", temp)
    }
  }

  fun updateInputs(inputs: ShooterPivotIOInputs)

  fun setAngleSetpoint(angle: Double): REVLibError

  fun setPIDF(pidf: PIDFGains): REVLibError

  fun getAngleOffset(): Double

  fun setAngleOffset(offsetAngle: Double): REVLibError

  fun stop()
}