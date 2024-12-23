package org.shockwave

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.littletonrobotics.urcl.URCL
import java.nio.file.Files
import java.nio.file.Paths

object Robot : LoggedRobot() {
  private val logFolderFound = Alert("Logging is enabled.", Alert.AlertType.kInfo)
  private val logFolderNotFound = Alert("Logging is disabled as the log folder doesn't exist!", Alert.AlertType.kWarning)
  private val tuningMode = Alert("Tuning mode is enabled.", Alert.AlertType.kInfo)
  private val tuningModeAtComp = Alert("Tuning mode is enabled in competition round!", Alert.AlertType.kWarning)
  private val garbageCollectorTimer = Timer()

  override fun robotInit() {
    Logger.recordMetadata("MavenGroup", MAVEN_GROUP)
    Logger.recordMetadata("MavenName", MAVEN_NAME)
    Logger.recordMetadata("BuildDate", BUILD_DATE)
    Logger.recordMetadata("GitSHA", GIT_SHA)
    Logger.recordMetadata("GitBranch", GIT_BRANCH)
    when (DIRTY) {
      0 -> Logger.recordMetadata("GitDirty", "All changes committed")
      1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes")
      else -> Logger.recordMetadata("GitDirty", "Unknown")
    }

    when (GlobalConstants.CURRENT_MODE) {
      GlobalConstants.Companion.Mode.REAL -> {
        Logger.addDataReceiver(NT4Publisher())
        PowerDistribution() // Enables power distribution logging.

        val logFolderPath = Paths.get(GlobalConstants.LOG_FOLDER_PATH)
        if (Files.notExists(logFolderPath)) Files.createDirectories(logFolderPath) // Create /log folder if it's a new RIO
        if (Files.exists(logFolderPath)) {
          logFolderFound.set(true)
          Logger.addDataReceiver(WPILOGWriter())
        } else {
          logFolderNotFound.set(true)
        }
      }

      GlobalConstants.Companion.Mode.SIM -> {
        Logger.addDataReceiver(NT4Publisher())
      }

      GlobalConstants.Companion.Mode.REPLAY -> {
        setUseTiming(false) // Run as fast as possible
        val logPath = LogFileUtil.findReplayLog()
        Logger.setReplaySource(WPILOGReader(logPath))
        Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
      }
    }

    Logger.registerURCL(URCL.startExternal())
    Logger.start()

    // Theoretically, this will never happen, but just in case...
    if (RobotContainer.isCompMatch() && GlobalConstants.TUNING_MODE) tuningModeAtComp.set(true)
    else tuningMode.set(GlobalConstants.TUNING_MODE)
  }

  override fun robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99)

    if (garbageCollectorTimer.advanceIfElapsed(5.0)) {
      System.gc()
    }

    CommandScheduler.getInstance().run()

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10)
  }

  override fun disabledPeriodic() {

  }

  override fun autonomousInit() {
    RobotContainer.autoChooser.get().schedule()
  }

  override fun autonomousPeriodic() {

  }

  override fun teleopInit() {

  }

  override fun teleopPeriodic() {

  }
}