package org.shockwave.subsystem.shooterpivot

data class ShooterPivotState(val name: String, val angle: Double) {
  companion object {
    val STARTING = ShooterPivotState("Starting", 2.5)
    val HOME = ShooterPivotState("Home", 30.0)
    val SUBWOOFER = ShooterPivotState("Subwoofer", 30.0)
    val SPIT = ShooterPivotState("Spit", 30.0)
    val AMP = ShooterPivotState("Amp", 25.0)
    val INTERPOLATED = ShooterPivotState("Interpolated", -1.0)
  }
}
