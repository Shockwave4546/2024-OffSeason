package org.shockwave.subsystem.swerve

import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import org.shockwave.subsystem.swerve.module.ModulePosition
import org.shockwave.utils.PIDFGains

class SwerveConstants {
  companion object {
    const val MAX_SPEED_METERS_PER_SECOND = 4.65
    const val MAX_ANGULAR_SPEED = 2 * Math.PI // radians per second

    // Distance between centers of right and left wheels on robot
    const val TRACK_WIDTH = 0.545 // m

    // Distance between front and back wheels on robot
    const val WHEEL_BASE = 0.545 // m

    val DRIVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
      ModulePosition.FRONT_LEFT.positionOffset,
      ModulePosition.FRONT_RIGHT.positionOffset,
      ModulePosition.BACK_LEFT.positionOffset,
      ModulePosition.BACK_RIGHT.positionOffset
    )

    // Driving Motor Prefix = 1x
    const val FRONT_LEFT_DRIVING_CAN_ID = 10
    const val FRONT_RIGHT_DRIVING_CAN_ID = 11
    const val BACK_LEFT_DRIVING_CAN_ID = 13
    const val BACK_RIGHT_DRIVING_CAN_ID = 14

    // Turning Motor Prefix = 2x
    const val FRONT_LEFT_TURNING_CAN_ID = 20
    const val FRONT_RIGHT_TURNING_CAN_ID = 21
    const val BACK_LEFT_TURNING_CAN_ID = 23
    const val BACK_RIGHT_TURNING_CAN_ID = 24

    const val GYRO_REVERSED = true

    const val DEFAULT_DRIVE_SPEED_MULTIPLIER = 0.85
    const val DEFAULT_ROT_SPEED_MULTIPLIER = 0.85

    val HEADING_PIDF = PIDFGains(0.85, 0.0, 0.05)
  }
}