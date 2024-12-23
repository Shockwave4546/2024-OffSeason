package org.shockwave.subsystem.shooterpivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.utils.TunableNumber
import org.shockwave.utils.TunablePIDF

class ShooterPivotSubsystem(private val pivot: ShooterPivotIO) : SubsystemBase() {
  private val inputs = ShooterPivotIO.ShooterPivotIOInputs()
  private var desiredState = ShooterPivotState.STARTING

  private val key = "ShooterPivot"
  private val pidf = TunablePIDF("$key/Tuning/", ShooterPivotConstants.GAINS)
  private val angleOffset = TunableNumber("$key/(1) AngleOffset", ShooterPivotConstants.ANGLE_OFFSET)

  override fun periodic() {
    pivot.updateInputs(inputs)
    Logger.processInputs(key, inputs)

    pidf.periodic(pivot::setPIDF) { value ->
      val clamped = MathUtil.clamp(value, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE)
      pivot.setAngleSetpoint(clamped)
      this.desiredState = ShooterPivotState("Manual", clamped)
    }

    TunableNumber.ifChanged(hashCode(), { values ->
      pivot.setAngleOffset(values[0])
    }, angleOffset)

    if (shouldStopPivot()) {
      ShooterPivotConstants.SHOULD_STOP_MOTOR_ALERT.text = "Shooter Pivot is at a dangerous angle, ${inputs.angle} > $ShooterPivotConstants.MAX_ANGLE"
      ShooterPivotConstants.SHOULD_STOP_MOTOR_ALERT.set(true)
    }

    Logger.recordOutput("$key/DesiredState/(1) Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/(2) Angle", desiredState.angle)
    Logger.recordOutput("$key/DesiredState/(3) At Goal", atDesiredState())
    Logger.recordOutput("$key/(2) Should Stop Pivot", shouldStopPivot())
  }

  fun setDesiredState(state: ShooterPivotState) {
    if (pidf.isManualMode()) return
    if (shouldStopPivot()) return

//    if (state === ShooterPivotState.INTERPOLATED) {
//      val distance = vision.getToSpeaker().distance
//      if (distance.isEmpty) return
//      this.desiredState = ShooterPivotState("Interpolated", ShooterPivotConstants.ANGLE_PREDICTOR.predict(distance.get()))
//    } else {
//      this.desiredState = state
//    }

    val clamped = MathUtil.clamp(desiredState.angle, ShooterPivotConstants.MIN_ANGLE, ShooterPivotConstants.MAX_ANGLE)
    pivot.setAngleSetpoint(clamped)
  }

  fun atDesiredState() = inputs.angle in desiredState.angle - ShooterPivotConstants.ANGLE_TOLERANCE..desiredState.angle + ShooterPivotConstants.ANGLE_TOLERANCE

  /**
   * If the Encoder is reading an angle that causes the pivot to go into the robot, it should stop.
   * These angles include [81, 360].
   *
   * @return whether the pivot should stop operating as to not break it.
   */
  private fun shouldStopPivot() = inputs.angle > ShooterPivotConstants.MAX_ANGLE
}