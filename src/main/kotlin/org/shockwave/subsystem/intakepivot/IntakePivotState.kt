package org.shockwave.subsystem.intakepivot

/**
 * Because the arm moves way too quick, the intermediate angle is a "solution" to slow it down before reaching the setpoint.
 */
data class IntakePivotState(val name: String, val angle: Double, val intermediateState: IntakePivotState?) {
  companion object {
    private val HOME_INTERMEDIATE = IntakePivotState("Home Intermediate", 20.0, null)
    val HOME = IntakePivotState("Home", 10.0, HOME_INTERMEDIATE)

    private val FLOOR_INTERMEDIATE = IntakePivotState("Floor Intermediate", 185.0, null)
    val FLOOR = IntakePivotState("Floor", 196.0, FLOOR_INTERMEDIATE)
  }
}