package org.shockwave.subsystem.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.subsystem.intake.commands.IdleIntakeCommand
import org.shockwave.utils.TunableBoolean

class IntakeSubsystem(private val intake: IntakeIO) : SubsystemBase() {
  private val inputs = IntakeIO.IntakeIOInputs()
  private var desiredState = IntakeState.IDLE
  private val key = "Intake"
  private val runIdle = TunableBoolean("$key/Tuning/RunIdle", true)

  init {
    defaultCommand = IdleIntakeCommand(this)
  }

  override fun periodic() {
    intake.updateInputs(inputs)

    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/DesiredState/1.Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/2.Duty Cycle", desiredState.dutyCycle)
    intake.setDutyCycle(desiredState.dutyCycle)
  }

  fun setDesiredState(desiredState: IntakeState) {
    this.desiredState = desiredState
  }

  fun isIdle() = runIdle.get()

  fun hasNote() = intake.hasNote()
}