package org.shockwave.subsystem.swerve

import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import org.shockwave.MotorConstants
import kotlin.math.hypot

class SwerveConstants {
  companion object {
    const val ODOMETRY_FREQUENCY = 100.0 // Hz
    const val MAX_SPEED_METERS_PER_SECOND = 4.65

    // Distance between centers of right and left wheels on robot
    const val TRACK_WIDTH = 0.545 // m
    // Distance between front and back wheels on robot
    const val WHEEL_BASE = 0.545 // m
    private val DRIVE_BASE_RADIUS = hypot(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
    val MAX_ANGULAR_SPEED_RAD_PER_SECOND = MAX_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS

    // Drive Motor Can ID Format = 1x
    const val FRONT_LEFT_DRIVE_ID = 10
    const val FRONT_RIGHT_DRIVE_ID = 11
    const val BACK_LEFT_DRIVE_ID = 13
    const val BACK_RIGHT_DRIVE_ID = 14

    // Turn Motor Can ID Format = 2x
    const val FRONT_LEFT_TURN_ID = 20
    const val FRONT_RIGHT_TURN_ID = 21
    const val BACK_LEFT_TURN_ID = 23
    const val BACK_RIGHT_TURN_ID = 24

    val AUTO_DRIVE_PID = PIDConstants(1.5, 0.0, 0.08)
    val AUTO_TURN_PID = PIDConstants(9.5, 0.0, 0.0)

    val MODULE_TRANSLATIONS = arrayOf(
      ModulePosition.FRONT_LEFT.positionOffset,
      ModulePosition.FRONT_RIGHT.positionOffset,
      ModulePosition.BACK_LEFT.positionOffset,
      ModulePosition.BACK_RIGHT.positionOffset
    )

    val PATH_PLANNER_ROBOT_CONFIG = RobotConfig(
      45.3592,  // Robot mass (kg)
      6.883,       // Robot MOI (kg m^2)
      ModuleConfig(
        Units.inchesToMeters(1.5),   // Wheel radius (m)
        4.65,             // Max speed (m/s)
        1.43,                     // Wheel COF (unitless)
        DCMotor.getNEO(1).withReduction(ModuleConstants.DRIVE_MOTOR_REDUCTION),
        MotorConstants.NEO_CURRENT_LIMIT.toDouble(),
        1
      ),
      *MODULE_TRANSLATIONS
    )

    val DRIVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
      ModulePosition.FRONT_LEFT.positionOffset,
      ModulePosition.FRONT_RIGHT.positionOffset,
      ModulePosition.BACK_LEFT.positionOffset,
      ModulePosition.BACK_RIGHT.positionOffset
    )
  }
}