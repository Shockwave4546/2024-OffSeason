package org.shockwave.subsystem.shooter

data class ShooterState(val name: String, val bottomRPS: Double, val topRPS: Double) {
  companion object {
    val STOPPED = ShooterState("Stopped", 0.0, 0.0)
    val AMP = ShooterState("Amp", 20.0, 12.0)
    val SUBWOOFER = ShooterState("Subwoofer", 60.0, 40.0)
    val IDLE = ShooterState("Idle", 30.0, 30.0)
    val PASS = ShooterState("Spit", 40.0, 50.0)
    val INTERPOLATED = ShooterState("Interpolated", -1.0, -1.0)
  }
}