package org.shockwave.subsystem.swerve.commands

import edu.wpi.first.wpilibj2.command.Command
import org.shockwave.subsystem.swerve.SwerveConstants
import org.shockwave.subsystem.swerve.SwerveSubsystem

class SetMaxSpeedCommand(private val swerve: SwerveSubsystem, private val driveMax: Double, private val rotMax: Double) : Command() {
  override fun execute() {
    swerve.setMaxSpeed(driveMax, rotMax)
  }

  override fun end(interrupted: Boolean) {
    swerve.setMaxSpeed(SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER, SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER)
  }
}