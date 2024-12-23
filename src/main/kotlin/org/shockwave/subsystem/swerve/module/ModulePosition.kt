package org.shockwave.subsystem.swerve.module

import edu.wpi.first.math.geometry.Translation2d
import org.shockwave.subsystem.swerve.SwerveConstants.Companion.TRACK_WIDTH
import org.shockwave.subsystem.swerve.SwerveConstants.Companion.WHEEL_BASE

/**
 * @param positionOffset The offset of the module from the center of the robot in meters.
 * @param angleOffset The angle offset of the module from the center of the robot in radians.
 */
enum class ModulePosition(val key: String, val positionOffset: Translation2d, val angleOffset: Double) {
  FRONT_LEFT("FrontLeft", Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), -Math.PI),
  FRONT_RIGHT("FrontRight", Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), -Math.PI / 2),
  BACK_LEFT("BackLeft", Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), Math.PI / 2),
  BACK_RIGHT("BackRight", Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), 0.0)
}