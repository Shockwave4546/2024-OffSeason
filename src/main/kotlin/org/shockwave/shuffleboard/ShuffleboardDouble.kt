package org.shockwave.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.littletonrobotics.junction.Logger

/**
 * A class representing a double value on the Shuffleboard dashboard.
 */
open class ShuffleboardDouble(tab: ShuffleboardTab, private val name: String, private val def: Double = DEFAULT_VALUE) :
  ShuffleboardValue() {
  private val widget = tab.add(name, def)

  /**
   * Constructs a new ShuffleboardDouble object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardDouble object
   */
  open fun withSize(length: Int, height: Int): ShuffleboardDouble {
    widget.withSize(length, height)
    return this
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardDouble object
   */
  open fun withPosition(x: Int, y: Int): ShuffleboardDouble {
    widget.withPosition(x, y)
    return this
  }

  /**
   * Sets the minimum and maximum values for the widget.
   *
   * @param min the minimum value for the widget
   * @param max the maximum value for the widget
   * @return the modified ShuffleboardDouble object
   */
  fun withMinMax(min: Double, max: Double): ShuffleboardDouble {
    widget.withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(mapOf("min" to min.toString(), "max" to max.toString()))
    return this
  }

  /**
   * Retrieves the current value of the ShuffleboardDouble object.
   *
   * @return the current value of the ShuffleboardDouble object
   */
  fun get() = widget.entry.getDouble(def)

  /**
   * Sets the value of the ShuffleboardDouble object.
   *
   * @param value the new value to set
   */
  fun set(value: Double) {
    Logger.recordOutput("DashboardInputs/$name", value)
    widget.entry.setDouble(value)
  }

  /**
   * Returns the Raw GenericEntry object associated with this ShuffleboardDouble.
   *
   * @return the Raw GenericEntry object
   */
  override fun getRaw() = widget.entry!!

  companion object {
    private const val DEFAULT_VALUE = 0.0
  }
}