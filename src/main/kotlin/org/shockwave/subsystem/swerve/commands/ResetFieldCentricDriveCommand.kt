package org.shockwave.subsystem.swerve.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import org.shockwave.subsystem.swerve.SwerveSubsystem
import org.shockwave.subsystem.vision.VisionSubsystem

class ResetFieldCentricDriveCommand(swerve: SwerveSubsystem, vision: VisionSubsystem) : InstantCommand({
  swerve.zeroGyro()
  vision.resetFieldOrientatedPose()
})