package org.shockwave.subsystem.intake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj.DigitalInput
import org.shockwave.subsystem.intake.IntakeConstants.Companion.MOTOR_CONFIG

class IntakeIOSpark : IntakeIO {
  private val motor = SparkMax(IntakeConstants.MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless)
  private val limitSwitch = DigitalInput(IntakeConstants.LIMIT_SWITCH_DIO_PORT)

  init {
    motor.configure(MOTOR_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    inputs.dutyCycle = motor.appliedOutput
    inputs.appliedVolts = motor.appliedOutput * motor.busVoltage
    inputs.current = motor.outputCurrent
    inputs.temp = motor.motorTemperature
    inputs.hasNote = hasNote()
  }

  override fun setDutyCycle(dutyCycle: Double) = motor.set(dutyCycle)

  override fun hasNote() = limitSwitch.get()
}