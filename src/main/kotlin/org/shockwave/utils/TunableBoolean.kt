package org.shockwave.utils

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.shockwave.GlobalConstants
import org.shockwave.RobotContainer

class TunableBoolean(key: String, private val defaultValue: Boolean = false) {
  private var dashboardBoolean: LoggedDashboardBoolean? = null
  private val lastHasChangedValues = hashMapOf<Int, Boolean>()

  init {
    if (GlobalConstants.TUNING_MODE && !RobotContainer.isCompMatch()) {
      dashboardBoolean = LoggedDashboardBoolean(key, defaultValue)
    }
  }

  fun get() = dashboardBoolean?.get() ?: defaultValue

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
    private const val TABLE_KEY = "Tuning"

    fun ifChanged(id: Int, action: (BooleanArray) -> Unit, vararg tunableBooleans: TunableBoolean) =
      tunableBooleans.filter { it.hasChanged(id) }.forEach { _ ->
        action(tunableBooleans.map { it.get() }.toBooleanArray())
      }

    fun ifChanged(id: Int, action: Runnable, vararg tunableBooleans: TunableBoolean) =
      ifChanged(id, { _: BooleanArray -> action.run() }, *tunableBooleans)
  }
}