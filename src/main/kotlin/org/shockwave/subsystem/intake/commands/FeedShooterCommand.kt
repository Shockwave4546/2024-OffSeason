package org.shockwave.subsystem.intake.commands

import edu.wpi.first.wpilibj2.command.Command
import org.shockwave.subsystem.intake.IntakeState
import org.shockwave.subsystem.intake.IntakeSubsystem

class FeedShooterCommand(private val intake: IntakeSubsystem) : Command() {
  init {
    addRequirements(intake)
  }

  override fun execute() = intake.setDesiredState(IntakeState.FEED)
  
  override fun end(interrupted: Boolean) {
    if (intake.isIdle()) intake.setDesiredState(IntakeState.IDLE)
    else intake.setDesiredState(IntakeState.STOPPED)
  }
}