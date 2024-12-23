package org.shockwave.subsystem.swervenew

import edu.wpi.first.math.geometry.Translation2d
import org.shockwave.subsystem.swerve.SwerveConstants.Companion.TRACK_WIDTH
import org.shockwave.subsystem.swerve.SwerveConstants.Companion.WHEEL_BASE

/**
 * @param positionOffset The offset of the module from the center of the robot in meters.
 * @param angleOffset The angle offset of the module from the center of the robot in radians.
 */
enum class ModulePosition(val key: String, val driveID: Int, val turnID: Int, val positionOffset: Translation2d, val angleOffset: Double) {
  FRONT_LEFT("FrontLeft", SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), -Math.PI),
  FRONT_RIGHT("FrontRight", SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), -Math.PI / 2),
  BACK_LEFT("BackLeft", SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), Math.PI / 2),
  BACK_RIGHT("BackRight", SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), 0.0)
}