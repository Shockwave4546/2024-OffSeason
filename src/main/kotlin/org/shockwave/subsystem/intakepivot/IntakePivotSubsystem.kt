package org.shockwave.subsystem.intakepivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.shockwave.utils.TunableNumber
import org.shockwave.utils.TunablePIDF

class IntakePivotSubsystem(private val pivot: IntakePivotIO) : SubsystemBase() {
  private val inputs = IntakePivotIO.IntakePivotIOInputs()
  private var desiredState = IntakePivotState.HOME

  private val key = "IntakePivot"
  private val pidf = TunablePIDF("$key/Tuning/", IntakePivotConstants.GAINS)

  private val angleOffset = TunableNumber("$key/7.AngleOffset", IntakePivotConstants.ANGLE_OFFSET)

  override fun periodic() {
    pivot.updateInputs(inputs)
    Logger.processInputs(key, inputs)

    pidf.periodic(pivot::setPIDF) { value ->
      val clamped = MathUtil.clamp(value, IntakePivotConstants.MIN_ANGLE, IntakePivotConstants.MAX_ANGLE)
      pivot.setAngleSetpoint(clamped)
      this.desiredState = IntakePivotState("Manual", clamped, null)
    }

    TunableNumber.ifChanged(angleOffset.hashCode(), { values ->
      pivot.setAngleOffset(values[0])
    }, angleOffset)

    if (shouldStopPivot()) {
      IntakePivotConstants.SHOULD_STOP_MOTOR_ALERT.text = "Intake Pivot is at a dangerous angle, ${inputs.angle} > $IntakePivotConstants.MAX_ANGLE"
      IntakePivotConstants.SHOULD_STOP_MOTOR_ALERT.set(true)
    }

    Logger.recordOutput("$key/DesiredState/1.Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/2.Angle", desiredState.angle)
    Logger.recordOutput("$key/DesiredState/3.At Goal", atDesiredState())
    Logger.recordOutput("$key/Should Stop Arm", shouldStopPivot())
  }

  fun setDesiredState(desiredState: IntakePivotState) {
    if (pidf.isManualMode()) return
    if (shouldStopPivot()) return

    val clamped = MathUtil.clamp(desiredState.angle, IntakePivotConstants.MIN_ANGLE, IntakePivotConstants.MAX_ANGLE)
    pivot.setAngleSetpoint(clamped)
    this.desiredState = desiredState
  }

  fun atDesiredState() = atDesiredState(desiredState)

  fun atDesiredState(state: IntakePivotState) = inputs.angle in state.angle - IntakePivotConstants.ANGLE_TOLERANCE..state.angle + IntakePivotConstants.ANGLE_TOLERANCE

  /**
   * If the Encoder is reading an angle that causes the arm to go into the robot, it should stop.
   * These angles include [198, 360].
   *
   * @return whether the arm should stop operating as to not break it.
   */
  private fun shouldStopPivot() = inputs.angle > IntakePivotConstants.MAX_ANGLE
}