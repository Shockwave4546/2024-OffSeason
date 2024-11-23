package org.shockwave.subsystem.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

class VisionConstants {
  companion object {
    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    val FRONT_CAMERA_TO_ROBOT = Transform3d(Translation3d(0.08, 0.15, 0.0), Rotation3d(0.0, 0.0, 55.0))
    val ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse()!!

    const val MAXIMUM_AMBIGUITY = 0.5
    const val FRONT_CAMERA_NAME = "FrontCamera"

    const val FIELD_LENGTH = 16.54175
    const val FIELD_WIDTH = 8.0137

    const val RED_SPEAKER_ID = 4
    const val BLUE_SPEAKER_ID = 7

    val HEADING_TOLERANCE = Units.degreesToRadians(2.0)

    const val SHOOTABLE_DISTANCE = 3.1

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     * Source: [...](https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java)
     */
    val STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0))!!

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     * Source: [...](https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java)
     */
    val VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10.0))!!
  }
}