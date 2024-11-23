package org.shockwave.subsystem.intake.commands

import edu.wpi.first.wpilibj2.command.Command
import org.shockwave.subsystem.intake.IntakeState
import org.shockwave.subsystem.intake.IntakeSubsystem

class IdleIntakeCommand(private val intake: IntakeSubsystem) : Command() {
  init {
    addRequirements(intake)
  }

  override fun execute() {
    if (!intake.isIdle()) {
      intake.setDesiredState(IntakeState.STOPPED)
      return
    }

    intake.setDesiredState(IntakeState.IDLE)
  }
}