package org.shockwave.shuffleboard

import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.littletonrobotics.junction.Logger

/**
 * A class representing a boolean value on the Shuffleboard dashboard.
 */
class ShuffleboardBoolean(tab: ShuffleboardTab, private val name: String, private val def: Boolean = DEFAULT_VALUE) :
  ShuffleboardValue() {
  private val widget = tab.add(name, def)

  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */
  init {
    widget.withWidget(BuiltInWidgets.kToggleButton)
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardBoolean object
   */
  fun withSize(length: Int, height: Int): ShuffleboardBoolean {
    widget.withSize(length, height)
    return this
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardBoolean object
   */
  fun withPosition(x: Int, y: Int): ShuffleboardBoolean {
    widget.withPosition(x, y)
    return this
  }

  /**
   * Retrieves the current value of the ShuffleboardBoolean object.
   *
   * @return the current value of the ShuffleboardBoolean object
   */
  fun get() = widget.entry.getBoolean(def)

  /**
   * Sets the value of the ShuffleboardBoolean object.
   *
   * @param value the new value to set
   */
  fun set(value: Boolean) {
    Logger.recordOutput("DashboardInputs/$name", value)
    widget.entry.setBoolean(value)
  }

  /**
   * Returns the Raw GenericEntry object associated with this ShuffleboardBoolean.
   *
   * @return the Raw GenericEntry object
   */
  override fun getRaw(): GenericEntry {
    return widget.entry
  }

  companion object {
    private const val DEFAULT_VALUE = false
  }
}