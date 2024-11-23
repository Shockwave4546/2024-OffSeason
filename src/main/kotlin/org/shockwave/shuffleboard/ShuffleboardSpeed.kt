package org.shockwave.shuffleboard

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

/**
 * A Shuffleboard control for adjusting the speed value.
 */
class ShuffleboardSpeed(tab: ShuffleboardTab, name: String, def: Double = DEFAULT_VALUE) :
  ShuffleboardDouble(tab, name, def) {
  /**
   * Creates a ShuffleboardSpeed widget with the given name and default value,
   * and adds it to the specified Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the widget to
   * @param name the name of the ShuffleboardSpeed widget
   * @param def the default value of the ShuffleboardSpeed widget
   */
  init {
    withMinMax(-1.0, 1.0)
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardDouble object
   */
  override fun withSize(length: Int, height: Int): ShuffleboardSpeed {
    super.withSize(length, height)
    return this
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardDouble object
   */
  override fun withPosition(x: Int, y: Int): ShuffleboardSpeed {
    super.withPosition(x, y)
    return this
  }

  companion object {
    private const val DEFAULT_VALUE = 0.0
  }
}