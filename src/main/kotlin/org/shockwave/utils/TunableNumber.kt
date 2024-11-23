package org.shockwave.utils

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import org.shockwave.GlobalConstants
import org.shockwave.RobotContainer

class TunableNumber(key: String, private val defaultValue: Double = 0.0) {
  private var dashboardNumber: LoggedDashboardNumber? = null
  private val lastHasChangedValues = hashMapOf<Int, Double>()

  init {
    if (GlobalConstants.TUNING_MODE && !RobotContainer.isCompMatch()) {
      dashboardNumber = LoggedDashboardNumber(key, defaultValue)
    }
  }

  fun get() = dashboardNumber?.get() ?: defaultValue

  fun hasChanged(id: Int): Boolean {
    val currentValue = get()
    val lastValue = lastHasChangedValues[id]
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues[id] = currentValue
      return true
    }

    return false
  }

  companion object {
    fun ifChanged(id: Int, action: (DoubleArray) -> Unit, vararg tunableNumbers: TunableNumber) =
      tunableNumbers.filter { it.hasChanged(id) }.forEach { _ ->
        action(tunableNumbers.map { it.get() }.toDoubleArray())
      }

    fun ifChanged(id: Int, action: Runnable, vararg tunableNumbers: TunableNumber) =
      ifChanged(id, { _: DoubleArray -> action.run() }, *tunableNumbers)
  }
}