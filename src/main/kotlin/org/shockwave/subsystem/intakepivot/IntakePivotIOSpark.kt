package org.shockwave.subsystem.intakepivot

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.shockwave.utils.PIDFGains

class IntakePivotIOSpark : IntakePivotIO {
  private val motor = SparkMax(IntakePivotConstants.MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless)
  private val encoder = motor.absoluteEncoder
  private val pid = motor.closedLoopController

  init {
    motor.configure(IntakePivotConstants.MOTOR_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
  }

  override fun updateInputs(inputs: IntakePivotIO.IntakePivotIOInputs) {
    inputs.angle = encoder.position
    inputs.appliedVolts = motor.appliedOutput * motor.busVoltage
    inputs.current = motor.outputCurrent
    inputs.temp = motor.motorTemperature
  }

  override fun setAngleSetpoint(angle: Double): REVLibError = pid.setReference(angle, SparkBase.ControlType.kPosition)

  override fun setPIDF(pidf: PIDFGains): REVLibError = motor.configure(
    SparkMaxConfig().apply(
      ClosedLoopConfig().pidf(pidf.p, pidf.i, pidf.d, pidf.ff)
    ),
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  // https://github.com/REVrobotics/REV-Software-Binaries/issues/8
  override fun getAngleOffset() = 0.0

  override fun setAngleOffset(offsetAngle: Double): REVLibError = motor.configure(
    SparkMaxConfig().apply {
      absoluteEncoder.zeroOffset(offsetAngle / IntakePivotConstants.ANGLE_CONVERSION_FACTOR)
    },
    SparkBase.ResetMode.kNoResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  override fun stop() = motor.stopMotor()
}