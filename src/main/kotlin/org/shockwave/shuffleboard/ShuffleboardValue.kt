package org.shockwave.shuffleboard

import edu.wpi.first.networktables.GenericEntry

/**
 * The ShuffleboardValue interface represents a shuffleboard value that can be displayed on the Shuffleboard dashboard.
 */
abstract class ShuffleboardValue {
  /**
   * Retrieves the raw GenericEntry object.
   *
   * @return The raw GenericEntry object.
   */
  abstract fun getRaw(): GenericEntry
}