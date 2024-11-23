package org.shockwave.subsystem.intake

data class IntakeState(val name: String, val dutyCycle: Double) {
  companion object {
    val STOPPED = IntakeState("Stopped", 0.0)
    val INTAKE = IntakeState("Intake", -1.0)
    val FEED = IntakeState("Feed", 1.0)
    val IDLE = IntakeState("Idle", -0.5)
  }
}