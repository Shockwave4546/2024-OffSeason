package org.shockwave.subsystem.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var hasNote = false
    var dutyCycle = 0.0
    var appliedVolts = 0.0
    var current = 0.0
    var temp = 0.0

    override fun toLog(table: LogTable) {
      table.put("(1) HasNote", hasNote)
      table.put("(2) DutyCycle", dutyCycle)
      table.put("(3) AppliedVolts", appliedVolts)
      table.put("(4) Current", current)
      table.put("(5) Temp", temp)
    }

    override fun fromLog(table: LogTable) {
      hasNote = table.get("(1) HasNote", hasNote)
      dutyCycle = table.get("(2) DutyCycle", dutyCycle)
      appliedVolts = table.get("(3) AppliedVolts", appliedVolts)
      current = table.get("(4) Current", current)
      temp = table.get("(5) Temp", temp)
    }
  }

  fun updateInputs(inputs: IntakeIOInputs)

  fun setDutyCycle(dutyCycle: Double)

  fun hasNote(): Boolean
}