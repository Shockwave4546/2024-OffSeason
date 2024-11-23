package org.shockwave.subsystem.swerve.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.shockwave.GlobalConstants
import org.shockwave.subsystem.swerve.SwerveSubsystem
import org.shockwave.subsystem.vision.VisionSubsystem

class SwerveDriveCommand(private val controller: CommandXboxController, private val swerve: SwerveSubsystem, private val vision: VisionSubsystem) : Command() {
  init {
    addRequirements(swerve)
  }

  override fun execute() {
    var rotSpeed = -MathUtil.applyDeadband(controller.rightX, GlobalConstants.DRIVE_DEADBAND)

    if (swerve.isAutoAlign()) {
      val rot = vision.getToSpeaker().rotation2d
      if (rot.isPresent) {
        rotSpeed = swerve.calculateSpeedForDesiredHeading(rot.get().radians)
      }
    }

    swerve.drive(
      -MathUtil.applyDeadband(controller.leftY, GlobalConstants.DRIVE_DEADBAND),
      -MathUtil.applyDeadband(controller.leftX, GlobalConstants.DRIVE_DEADBAND),
      rotSpeed,
      swerve.isFieldRelative(),
      useConstDriveSpeed = false,
      useConstRotSpeed = swerve.isAutoAlign()
    )
  }
}