package org.shockwave.subsystem.shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.BOT_GAINS
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.BOT_RPS_PREDICTOR
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.MAX_RPS
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.MIN_RPS
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.TOP_GAINS
import org.shockwave.subsystem.shooter.ShooterConstants.Companion.TOP_RPS_PREDICTOR
import org.shockwave.subsystem.vision.VisionSubsystem
import org.shockwave.utils.TunablePIDF

class ShooterSubsystem(private val shooter: ShooterIO, private val vision: VisionSubsystem) : SubsystemBase() {
  private val inputs = ShooterIO.ShooterIOInputs()
  private var desiredState = ShooterState.STOPPED

  private val botPIDF = TunablePIDF("Shooter/Tuning/Bot/", BOT_GAINS)
  private val topPIDF = TunablePIDF("Shooter/Tuning/Top/", TOP_GAINS)

  override fun periodic() {
    shooter.updateInputs(inputs)
    Logger.processInputs("Shooter", inputs)

    botPIDF.periodic(shooter::setBotPIDF) { value ->
      val clamped = MathUtil.clamp(value, MIN_RPS, MAX_RPS)
      shooter.setBottomVelocitySetpoint(clamped)
      this.desiredState = ShooterState("Manual", clamped, this.desiredState.topRPS)
    }

    topPIDF.periodic(shooter::setTopPIDF) { value ->
      val clamped = MathUtil.clamp(value, MIN_RPS, MAX_RPS)
      shooter.setTopVelocitySetpoint(clamped)
      this.desiredState = ShooterState("Manual", this.desiredState.bottomRPS, clamped)
    }

    Logger.recordOutput("Shooter/DesiredState/(1) Name", desiredState.name)
    Logger.recordOutput("Shooter/DesiredState/(2) Bot RPS", desiredState.bottomRPS)
    Logger.recordOutput("Shooter/DesiredState/(3) Top RPS", desiredState.topRPS)
    Logger.recordOutput("Shooter/DesiredState/(4) At Goal", atDesiredState())
  }

  fun setDesiredState(state: ShooterState) {
    if (topPIDF.isManualMode() || botPIDF.isManualMode()) return

    if (state === ShooterState.INTERPOLATED) {
      val distance = vision.getToSpeaker().distance
      if (distance.isEmpty) return
      this.desiredState = ShooterState(
        "Interpolated",
        BOT_RPS_PREDICTOR.predict(distance.get()),
        TOP_RPS_PREDICTOR.predict(distance.get())
      )
    } else {
      this.desiredState = state
    }

    val bottomClamped = MathUtil.clamp(desiredState.bottomRPS, MIN_RPS, MAX_RPS)
    val topClamped = MathUtil.clamp(desiredState.topRPS, MIN_RPS, MAX_RPS)
    shooter.setBottomVelocitySetpoint(bottomClamped)
    shooter.setTopVelocitySetpoint(topClamped)
  }

  private fun bottomAtDesiredState() =
    inputs.bottomRPS in desiredState.bottomRPS - ShooterConstants.RPS_TOLERANCE..desiredState.bottomRPS + ShooterConstants.RPS_TOLERANCE

  private fun topAtDesiredState() =
    inputs.topRPS in desiredState.topRPS - ShooterConstants.RPS_TOLERANCE..desiredState.topRPS + ShooterConstants.RPS_TOLERANCE

  fun atDesiredState() = bottomAtDesiredState() && topAtDesiredState()
}