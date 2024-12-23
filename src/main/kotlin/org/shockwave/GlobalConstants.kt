package org.shockwave

import edu.wpi.first.wpilibj.RobotBase

class GlobalConstants {
  companion object {
    enum class Mode {
      /** Running on a real robot.  */
      REAL,

      /** Running a physics simulator.  */
      SIM,

      /** Replaying from a log file.  */
      REPLAY
    }

    const val TUNING_MODE = true
    const val DRIVER_CONTROLLER_PORT = 0
    const val OPERATOR_CONTROLLER_PORT = 1
    const val DRIVE_DEADBAND = 0.02
    const val LOG_FOLDER_PATH = "/U/logs/"
    val CURRENT_MODE = if (RobotBase.isReal()) Mode.REAL else Mode.SIM
  }
}