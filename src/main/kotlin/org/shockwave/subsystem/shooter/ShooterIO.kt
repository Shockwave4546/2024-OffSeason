package org.shockwave.subsystem.shooter

import com.revrobotics.REVLibError
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.shockwave.utils.PIDFGains

interface ShooterIO {
  class ShooterIOInputs : LoggableInputs {
    var bottomRPS = 0.0
    var bottomAppliedVolts = 0.0
    var bottomCurrent = 0.0
    var bottomTemp = 0.0

    var topRPS = 0.0
    var topAppliedVolts = 0.0
    var topCurrent = 0.0
    var topTemp = 0.0

    override fun toLog(table: LogTable) {
      table.put("Bot/(1) RPS", bottomRPS)
      table.put("Bot/(2) Volts", bottomAppliedVolts)
      table.put("Bot/(3) Current", bottomCurrent)
      table.put("Bot/(4) Temp", bottomTemp)

      table.put("Top/(1) RPS", topRPS)
      table.put("Top/(2) Volts", topAppliedVolts)
      table.put("Top/(3) Current", topCurrent)
      table.put("Top/(4) Temp", topTemp)
    }

    override fun fromLog(table: LogTable) {
      bottomRPS = table.get("Bot/(1) RPS", bottomRPS)
      bottomAppliedVolts = table.get("Bot/(2) Volts", bottomAppliedVolts)
      bottomCurrent = table.get("Bot/(3) Current", bottomCurrent)
      bottomTemp = table.get("Bot/(4) Temp", bottomTemp)

      topRPS = table.get("Top/(1) RPS", topRPS)
      topAppliedVolts = table.get("Top/(2) Volts", topAppliedVolts)
      topCurrent = table.get("Top/(3) Current", topCurrent)
      topTemp = table.get("Top/(4) Temp", topTemp)
    }
  }

  fun updateInputs(inputs: ShooterIOInputs)

  fun setBottomVelocitySetpoint(rps: Double): REVLibError

  fun setBotPIDF(pidf: PIDFGains): REVLibError

  fun stopBot()

  fun setTopVelocitySetpoint(rps: Double): REVLibError

  fun setTopPIDF(pidf: PIDFGains): REVLibError

  fun stopTop()
}